/**************************
*
*	File Moteino_lora.ino
*	V 0.1 - Sept 2016
*	Author - E. Havet - https://github.com/sosandroid
*	License LGPL http://www.gnu.org/licenses/lgpl.html
*
* Description
* -----------
*	This code is part my tests in a larger project : a hot air balloon monitoring system. The constraints are :
*	    - Tramsitter form factor as compact as possible
*	    - LOS from 20m up to 10 km for transmition. Sometimes noLOS.
*	    - Hot working environment for some sensors(> 100°C)
*	    - As low power as possible
*	
*	Moteino board meets 3 out of 4 criteria : small form factor, LoRa ready throufh RFM95W, low power.
*	
*	
*	This file aims to test the Moteino + RFM95W + the Radiohead library RH_RF95 combination
*	We are trying to use
*	  - Custom modem settings									=> passed
*	  - Headers setting & reading								=> failed
*	  - Payload defined as a Struct to ease data manipulation	=> passed
*	
*	This file is made as generic as possible with no direct link to the larger projet. This makes it usable by anyone
*	Feel free to use it, modify it and share it.
*
*
* RH_RF95 Library
* ---------------
*	We use the version 1.62 of the Lib.
*	
*	This library aims to ease the use of the RFM95W module. However, it seems a bit complicated for beginners such as me. 
*	Time and readings helps a lot as the documentation is quite complete. The minimal must read :
*	  - The Semtech SX1276 datasheet for modem custom config (p.113 mainly) http://www.semtech.com/images/datasheet/sx1276.pdf
*	  - Library documentation (RH_RF95 and others drivers as there is useful code patterns everywhere) http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
*	  - LowPowerLab Moteino page about LoRa : https://lowpowerlab.com/moteino/#lora
*	  - RFM95W datasheet : http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf
*	  
*	All messages are formated that way as a default by the Radiohead Library in LoRa Mode
*	  - 8 symbol PREAMBLE - chip default
*	  - Explicit header with header CRC (handled internally by the radio)
*	  - 4 octets HEADER: (TO, FROM, ID, FLAGS)
*	  - 0 to 251 octets DATA 
*	  - CRC (handled internally by the radio)
*
* France's frequencies regulation
* -------------------------------
*   document ART n°02-935 et 02-939 http://www.anfr.fr/fileadmin/mediatheque/documents/tnrbf/Annexe_A7.pdf & http://www.arcep.fr/fileadmin/uploads/tx_gsavis/06-0841.pdf
*   863.0 to 865.0 MHz @ 25 mW  (~14 dBm) channel max width not regulated
*   865.0 to 865.6 MHz @ 100 mW (~20 dBm) channel max width 200 Khz - the 2 documents do not describe the same limitations
*   865.6 to 867.6 MHz @ 2000 mW (~33 dBm) channel max width 200 Khz - the 2 documents do not describe the same limitations
*   867.6 to 868.0 MHz @ 500 mW (~27 dBm) channel max width 200 Khz - the 2 documents do not describe the same limitations
*   868.0 to 869.2 MHz @ 25 mW  (~14 dBm) channel max width not regulated
*   869.3 to 869.4 MHz @ 10 mW (10 dBm) channel max width 25 Khz
*   869.4 to 869.65 MHz @ 500 mW (~27 dBm) channel max width 25Khz or whole band
*   869.7 to 870.00 MHz @ 25 mW (~14 dBm) channel max width not regulated
* 
* Testing
* -------
*	Tested against
*		- Arduino IDE 1.6.11 : Passed
*		- Codebender.cc : Failed, probably do not support properly radiohead lib
*		- Moteino R4 : to be tested as soon as I got the boards
*		- Moteino Mega : Passed
* 
* *****************************/

#include <SPI.h>
#include <RH_RF95.h>

//comment if you are using a Moteino R4 or other board
#define MOTEINOMEGA
//comment if you are using a Moteino Mega or other board
//#define MOTEINOR4
//Comment if you are compiling the transmitter side
//#define RECEIVER
//Comment if you are compiling the receiver side
#define TRANSMITER

#ifdef MOTEINOMEGA
	//MoteinoMega
	#define MODEM_SSPIN 4
	#define MODEM_INTERRUPTPIN 2
	#define PINLED 15
#else
	//Moteino R4
	#define MODEM_SSPIN 10
	#define MODEM_INTERRUPTPIN 2
	#define PINLED 9
#endif

//delay between transmissions
#define LOOP_DELAY 5000
//RF middle frequency
#define RF_FREQ 869.5
//#define RF_FREQ 915.0
//TX poxer in dBm (from 5 to 23)
#define RF_TXPOWER 13
//TX timeout
#define RF_TXTIMEOUT 12000
#define SERIAL_BAUD 9600
#define BLINKTIME 200



// Used with headerTo & headerFrom bytes - we do not use one of the managers provided by Radiohead lib to understand the way things work.
#define GATEWAY_ADDRESS 0x01
#define CLIENT_ADDRESS 0x09

// headerID used as message type
#define HEADERID_MAINDATA 0x01 //position, altitude, temp
#define HEADERID_STARTPOS 0x02 //start position & altitude
#define HEADERID_STARTTIME 0x03 //start time
#define HEADERID_TEMP 0x04 //stand alone temperature



/******************
* Modem Conf
*******************/
/******************
*	Used the loRa Calculator from Semtech to get values of link budget, power budget
*	Modem SX1276 settings 
*	   BandWidth 62.5 Khz,				(Default RH_RF95 125 Khz)
*	   Coding rate 4/8,					(Default RH_RF95 4/5)
*	   Explicit headers,				(Default RH_RF95 explicit)
*	   Spreading factor 2048 (11)		(Default RH_RF95 128 - 7)
*	   TX continuous disabled			(Default RH_RF95 disabled)
*	   CRC enabled						(Default RH_RF95 enabled)
*	   Low data rate optimize enabled	(Default RH_RF95 disabled)
*	   LNA gain set by register			(Default RH_RF95)
*	
*	Symbol Time 32,77 ms / receiver sensitivity -137,5 dBm
*	Transmission time for 13 bytes : 1450 ms
*	             3 bytes : 925  ms
*	             2 bytes : 925  ms
*	             0 bytes : 664  ms
*	
*	Go to SF 12 => receiver sensitity goes to -140 dBm, transmission time x2
*	Go to BW 31.25 Khz => receiver sensitity goes to -140.6 dBm, transmission time x2
***********************/

#define REGMODEMCONFIG1 0b01101000	//RegModemConfig1(0x1D)
									// 7..4   b0110  62.5 KHz // b0100 31.25 KHz
									// 3..1   b100 => 4/8
									// ..0    b0 => implicit header mode disabled => explicit
									// => 0x68
              
#define REGMODEMCONFIG2 0b10110100	//RegModemConfig2(0x1E)
									// 7..4   b1011 => Spreading factor 11 (2^11)  // b1100 for SF12
									// ..3    b0 => Tx continuous mode disabled
									// ..2    b1 => CRC enabled
									// 1..0   b00 => chip default config
									// => 0xB4
                  
#define REGMODEMCONFIG3 0b00001000	// RegModemConfig3(0x26)
									// 7..4   b0000 unused
									// ..3    b1 Low data rate optimize enabled // advised for symbol time > 16ms // LoRa Calculator needed here
									// ..2    b0 Auto gain LNA disabled
									// 1..0   b00 reserved 
									// => 0x08

static const RH_RF95::ModemConfig confModem = {REGMODEMCONFIG1, REGMODEMCONFIG2, REGMODEMCONFIG3};


/******************
* Struct definitions
*******************/                
typedef struct {
	double    data1;
	double    data2;
	double    data3;
	uint8_t   data4;
} payload;        // 1 + 4 + 4 + 4 = 13 bytes atmega368p / 1284p
//RH_RF95::send() can not use payload type as input

typedef union {
	payload structuredData;
	uint8_t txData[sizeof(payload)];
} unifiedData;
//This union transforms the payload into an array of bytes for RH_RF95::send()

unifiedData mydata;
//mydata.txData goes OTA
//mydata.structuredData is used by the sketch on each side of the RF link.

/**********************************/


RH_RF95 rf95(MODEM_SSPIN, MODEM_INTERRUPTPIN);
bool new_data = false;
int txData_size = sizeof(mydata.txData);

void setup()
{
	pinMode(PINLED, OUTPUT);
	digitalWrite(PINLED, LOW);
	Serial.begin(SERIAL_BAUD);
	while (!Serial) ; // Wait for serial port to be available
	
	if (!rf95.init()) {
		Serial.println("Something wrong with modem init !");
		Serial.println("Program stops here");
		while(true);
	}

	rf95.setFrequency(RF_FREQ);
	rf95.setModemRegisters(&confModem);
	//rf95.setHeaderId(HEADERID_MAINDATA); //prevent data to be send & / or received
	
	#ifdef TRANSMITER
		rf95.setTxPower(RF_TXPOWER, false);
		//rf95.setHeaderFrom(CLIENT_ADDRESS); //prevent data to be send & / or received
		//rf95.setHeaderTo(GATEWAY_ADDRESS); //prevent data to be send & / or received

		mydata.structuredData.data1 = 0x00;
		mydata.structuredData.data2 = 0xFFFF;
		mydata.structuredData.data3 = 0x1234;
		mydata.structuredData.data4 = 0x01;

		Serial.println("RFM95W transmitter ready to work. Please check serial output on the receiver side for Data & RSSI");
	#endif
	
	#ifdef RECEIVER
		//rf95.setHeaderFrom(GATEWAY_ADDRESS);  //prevent data to be send & / or received
		//rf95.setHeaderTo(CLIENT_ADDRESS);  //prevent data to be send & / or received

		mydata.structuredData.data1 = 0x00;
		mydata.structuredData.data2 = 0x00;
		mydata.structuredData.data3 = 0x00;
		mydata.structuredData.data4 = 0x00;
		
		Serial.println("RFM95W receiver ready to work. Data should arrive soon");
	#endif
	
	//rf95.setModeIdle();
	//End of setup - blink twice
	blinkled();
	blinkled();
  
}

void loop()
{
	#ifdef TRANSMITER
		//update data & send every LOOP_DELAY if transmitter
		updateData();
		sendData();
		delay(LOOP_DELAY);
	#endif
	
	#ifdef RECEIVER
		//listen for new data and print to serial if needed
		recvData();
		printData();
	#endif
  
}

/******************************************/
/****** Others functions              *****/
/******************************************/

void updateData() {
	//modifies data before sending
	//data1 increases, data2 decreases, data3 is unchanged, data4 is bit shifted left
	mydata.structuredData.data1 += 0x01;
	mydata.structuredData.data2 -= 0x01;
	//mydata.structuredData.data3 = 0x1234; //we do not change this value
	mydata.structuredData.data4 = mydata.structuredData.data4 << 1;
}

void sendData() {

	//send data OTA and call sum-up function

	bool rfok;
	unsigned long timing = millis();
	
	rf95.waitPacketSent();
	rfok = rf95.send(mydata.txData, txData_size);
	while(!rf95.waitPacketSent(RF_TXTIMEOUT));
	sumupTransmission(millis() - timing, rfok);
	if (rfok) blinkled(); //if data transmitted, blink
	//rf95.setModeIdle();
}

void recvData() {
	// checks for received packest and updates mydata.txData acordingly.
	uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
	uint8_t len = sizeof(buf);
	uint8_t HFrom;
	uint8_t HId;
	
	if(rf95.recv(buf, &len)) {
		//rf95.available();
		//rf95.recv(buf, &len);
		//HTo = rf95.headerTo();//prevent data to be send & / or received
		//HFrom = rf95.headerFrom();//prevent data to be send & / or received
		//HId = rf95.headerId(); //prevent data to be send & / or received
		
		//if((HFrom == CLIENT_ADDRESS) && (HId == HEADERID_MAINDATA)) { 	//check sender and data type - does not work
				for (uint8_t i = 0; i < txData_size; i++) {
					mydata.txData[i] = buf[i];							//probably not the best way to work
				}
			new_data = true;
			
		//}
		blinkled(); // blink each received packet whatever data is
		printData();
	}
	
}

void printData() {
	//outputs receives data to Serial
	if (new_data) {
		Serial.print("Data1: ");
		Serial.print(mydata.structuredData.data1, DEC);
		Serial.print(" Data2: ");
		Serial.print(mydata.structuredData.data2, HEX);
		Serial.print(" Data3: ");
		Serial.print(mydata.structuredData.data3, HEX);
		Serial.print(" Data4: ");
		Serial.print(mydata.structuredData.data4, DEC);
		Serial.print(" RSSI: ");
		Serial.println(rf95.lastRssi(), DEC);
	}

	new_data = false;
}

void sumupTransmission(unsigned long timing, bool rfstatus) {
	//outputs a sum-up of the transmitted data to Serial
	(rfstatus) ? Serial.print("OK  ") : Serial.print("NOK ");
	Serial.print("Data1: ");
	Serial.print(mydata.structuredData.data1, DEC);
	Serial.print(" | ");
	Serial.print(txData_size, DEC);
	Serial.print(" bytes sent in ");
	Serial.print(timing, DEC);
	Serial.println("ms");	
}

void blinkled() {
	digitalWrite(PINLED, HIGH);
	delay(BLINKTIME);
	digitalWrite(PINLED, LOW);
	delay(BLINKTIME);
}