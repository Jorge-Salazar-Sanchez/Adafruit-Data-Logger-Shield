/*
The Adafruit's Data Logging Shield for Arduino saves data to files on any FAT16- or FAT32-formatted SD card 
in a way that can be read later by most plotting, spreadsheet, or analysis programs.

The Real Time Clock (RTC) chip timestamps all the data with the current TIME, so that one may know precisely what happened


Created by Adafruit Industries
Edited and modified by Jorge Salazar Sanchez
in May 2019

*/


// SD library to store the data (added) //
#include <SD.h>


#include <Wire.h>
#include <SPI.h>             
#include <LoRa.h>


// RTClib library to control and use the internal RTC (Real Time Clock) (added) //
#include "RTClib.h"


// The RTC we'll be using is the DS1307 (added)// 
RTC_DS1307 rtc;


// Character array of 7 rows and 12 columns (added) //
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


const int co2Addr = 0x68; // This is the default address of the CO2 sensor, 7bits shifted left
const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends


// Labels used to identify the reading values (added) //
String labels = "DATE, DAY, TIME, CO2, TEMP, RH"; 


// Setting up a file (myfile) to read and write the data (added) //
File myFile;



///////////////////////////////////////////////////////////////////
// Setting up Communications (SETUP)
//////////////////////////////////////////////////////////////////

void setup() {

Wire.begin(); //Initiate the Wire library
rtc.begin(); //Initializes the internal RTC
Serial.begin(9600); //Sets the data rate in bits per second (baud) for serial data transmission


///////////////////////////////////////////////////////////////////
// Setting up Serial Port (added)
//////////////////////////////////////////////////////////////////

Serial.println(F("Type any character and press enter to start"));
while (!Serial.available());
Serial.println(F("Serial Port Initialied"));
Serial.println();
delay(1000);


///////////////////////////////////////////////////////////////////
// Setting up SD Card (added)
//////////////////////////////////////////////////////////////////

Serial.print("Initializing SD card...  ");
delay(1000);
Serial.println("");


// Check if the SD Card is in the holder shield (added) //

if (!SD.begin(5)) {
    Serial.println(F("Card failed, or not present"));
    Serial.println(F("Please, insert the SD card or check the CS pin, and then press any key to start again"));
    Serial.println(F(""));
    Serial.end();
    Serial.flush();
    Serial.begin(9600);
    while (!SD.begin(5)); //Check if the SD card is inside the holder shield (Pin 5 should then be ACTIVE)
    while (!Serial.available()); //A character must be introduced again and press enter to keep on
    delay(3000);
    Serial.println(F("Initializing SD card again... "));
    delay(2000);
    Serial.print(F("SD Card initialized!!!"));
    delay(1500);


// Open the CSV file to write the labels of each variables (CO2, Temp, RH) (added) //
 
 SD.remove("logger.csv");  
 myFile = SD.open("logger.csv", FILE_WRITE);


  if (myFile) {  
    myFile.println(labels);
      
    // close the file:
    myFile.close();
   
    Serial.println('\n');
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("Error Writing Labels"));
  }

// Close the file once the labels are already written (added) //
  
    Serial.println('\n');
    
}


// If not, if the SD Card is already in the holder shield (added) //

 else {
    delay(2000);
    Serial.print(F("SD Card initialized!!!"));


// Open the CSV file to write the labels of each variables (CO2, Temp, RH) (added)//

 SD.remove("logger.csv");  
 myFile = SD.open("logger.csv", FILE_WRITE);

  if (myFile) {  
    myFile.println(labels);
      
    // close the file:
    myFile.close();
    Serial.println('\n');
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("Error Writing Labels"));
  }
}


delay(2500);
Serial.println(F(""));
delay(2000);
 

//////////////////////////////////////////
// Setting up Real Time Clock Chip (added)
//////////////////////////////////////////

if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }

if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
}
 else {
    Serial.println(F("RTC is running!"));
    
// following line sets the RTC to the date & time this sketch was compiled

   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   //rtc.adjust(DateTime((2020, 3, 21), F(0, 0, 0)));
    
// This line sets the RTC with an explicit date & time, for example to set
// January 21, 2014 at 3am you would call:
   //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }


pinMode(LED_BUILTIN, OUTPUT); // We will use this pin (PIN 13 by default) as a read‐indicator
Serial.println(F("LoRa duplex config for CO2 Sensor"));
Serial.println(F("Initializing LoRa....."));
LoRa.setPins(csPin, resetPin, irqPin);
 if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println(F("LoRa init failed. Check your connections."));
    while (true);                       // if failed, do nothing
  }
Serial.println(F(".....LoRa init succeeded."));
Serial.println(F("")); //Just to make it nice
Serial.println(F("")); //Just to make it nice
Serial.println(F("")); //Just to make it nice

}


///////////////////////////////////////////////////////////////////
// Function : void wakeSensor()
// Executes : Sends wakeup commands to K33 sensors.
// Note : THIS COMMAND MUST BE MODIFIED FOR THE SPECIFIC AVR YOU ARE USING 
// THE REGISTERS ARE HARD‐CODED 
///////////////////////////////////////////////////////////////////

void wakeSensor() {
// This command serves as a wakeup to the CO2 sensor, for K33‐ELG/BLG Sensors Only

// You'll have the look up the registers for your specific device, but the idea here is simple: 
// 1. Disabled the I2C engine on the AVR
// 2. Set the Data Direction register to output on the SDA line
// 3. Toggle the line low for ~1ms to wake the micro up. Enable I2C Engine
// 4. Wake a millisecond.

  TWCR &= ~(1<<2); // Disable I2C Engine
  DDRC |= (1<<4); // Set pin to output mode
  PORTC &= ~(1<<4); // Pull pin low
  delay(1);
  PORTC |= (1<<4); // Pull pin high again
  TWCR |= (1<<2); // I2C is now enabled
  delay(1);
}


////////////////////////////////////////////////////////////////////// 
// Function : void initPoll()
// Executes : Tells sensor to take a measurement.
// Notes    : A fuller implementation would read the register back and
//            ensure the flag was set, but in our case we ensure the poll
//            period is >25s and life is generally good.
// ///////////////////////////////////////////////////////////////////

void initPoll() {
  Wire.beginTransmission(co2Addr);
  Wire.write(0x11);
  Wire.write(0x00);
  Wire.write(0x60);
  Wire.write(0x35);
  Wire.write(0xA6);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(co2Addr, 2);
  byte i = 0;
  byte buffer[2] = {0, 0};
  while(Wire.available()) {
      buffer[i] = Wire.read();
      i++;
} }


/////////////////////////////////////////////////////////////////// 
// Function : double readCo2()
// Returns : The current CO2 Value, ‐1 if error has occured 
///////////////////////////////////////////////////////////////////

double readCo2() {
int co2_value = 0;
// We will store the CO2 value inside this variable. 
digitalWrite(LED_BUILTIN, HIGH);
// On most Arduino platforms this pin is used as an indicator light.


//////////////////////////
  /* Begin Write Sequence */
//////////////////////////

Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.write(0x2A);
  Wire.endTransmission();

 /*
     We wait 10ms for the sensor to process our command.
     The sensors's primary duties are to accurately
     measure CO2 values. Waiting 10ms will ensure the
     data is properly written to RAM
*/
delay(20);


  /////////////////////////
  /* Begin Read Sequence */
  /////////////////////////

  /*
   Since we requested 2 bytes from the sensor we must
     read in 4 bytes. This includes the payload, checksum,
     and command status byte.
     
   */

Wire.requestFrom(co2Addr, 4);
byte i = 0;
byte buffer[4] = {0, 0, 0, 0};

/*
Wire.available() is not nessessary. Implementation is obscure but we leave it in here for portability and to future proof our code
*/

while(Wire.available()) {
    buffer[i] = Wire.read();
    i++;
}

co2_value = 0;
co2_value |= buffer[1] & 0xFF;
co2_value = co2_value << 8;
co2_value |= buffer[2] & 0xFF;
byte sum = 0;                             //Checksum Byte
sum = buffer[0] + buffer[1] + buffer[2];  //Byte addition utilizes overflow
if(sum == buffer[3]) {
    // Success!
    digitalWrite(LED_BUILTIN, LOW);
    return ((double) co2_value / (double) 1);
  }
  else {
   // Failure!
   /*
        Checksum failure can be due to a number of factors,
        fuzzy electrons, sensor busy, etc.
  */
      digitalWrite(LED_BUILTIN, LOW);
      return (double) -1;
  }
}

/////////////////////////////////////////////////////////////////// 
// Function : double readTemp()
// Returns : The current Temperture Value, ‐1 if error has occured 
///////////////////////////////////////////////////////////////////

double readTemp() {
  int tempVal = 0;
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x12);
  Wire.write(0x34);
  Wire.endTransmission();
  delay(20);

Wire.requestFrom(co2Addr, 4);
byte i = 0;
byte buffer[4] = {0, 0, 0, 0};
while(Wire.available()) {
    buffer[i] = Wire.read();
    i++;
}
tempVal = 0;
tempVal |= buffer[1] & 0xFF;
tempVal = tempVal << 8;
tempVal |= buffer[2] & 0xFF;
byte sum = 0;                                //Checksum Byte
sum = buffer[0] + buffer[1] + buffer[2];     //Byte addition utilizes overflow

  if(sum == buffer[3]) {
      digitalWrite(LED_BUILTIN, LOW);
      return ((double) tempVal / (double) 100);
}
else {
      digitalWrite(LED_BUILTIN, LOW);
return -1; 
}
}
  
/////////////////////////////////////////////////////////////////// 
// Function : double readRh()
// Returns : The current Rh Value, ‐1 if error has occured 
///////////////////////////////////////////////////////////////////

double readRh() {
   int tempVal = 0;
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x14);
  Wire.write(0x36);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(co2Addr, 4);
  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};
  while(Wire.available()) {
      buffer[i] = Wire.read();
      i++;
}

tempVal = 0;
tempVal |= buffer[1] & 0xFF;
tempVal = tempVal << 8;
tempVal |= buffer[2] & 0xFF;
byte sum = 0;                             //Checksum Byte
sum = buffer[0] + buffer[1] + buffer[2];  //Byte addition utilizes overflow
  if(sum == buffer[3]) {
      digitalWrite(LED_BUILTIN, LOW);
      return (double) tempVal / (double) 100;
}
else {
      digitalWrite(LED_BUILTIN, LOW);
return -1; }
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println(F("error: message length does not match length"));
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println(F("This message is not for me."));
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// Read, save, and print values on both the SD Card and the Serial Monitor through to LOOP function
///////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  // We keep the sample period >25s or so, else the sensor will start ignoring sample requests. 
  
  wakeSensor();
  initPoll();
  
  delay(16000);
  
  wakeSensor();
  float tempValue = readTemp();
  
  delay(20);
  wakeSensor();
  float rhValue = readRh();
  
  delay(20);
  wakeSensor();
  float co2Value = readCo2();
  
if(co2Value >= 0) {




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The NOW variable is a new object variable used to convert the returned seconds from the now() function into months, days, and year (added)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  DateTime now = rtc.now();
  
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print('/');
        Serial.print(now.year(), DEC);
        Serial.print(' ');
        Serial.print(" (");
        Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
        Serial.print(") ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.println(now.second(), DEC);
        Serial.print(F("CO2: "));
        Serial.print(co2Value);
        Serial.print(F("ppm Temp: "));
        Serial.print(tempValue);
        Serial.print(F("C Rh: "));
        Serial.print(rhValue);
        Serial.println(F("%"));
        Serial.println(F("Sending data to the receiver LoRa..."));
        String message = "CO2: " + String(co2Value) + " ppm Temp: " + String(tempValue) + " C Rh: " + String(rhValue) + "%" + "     " + String(now.month(), DEC) + String('/') + String(now.day(), DEC) + String('/') + String(now.year(), DEC) + " " + String(" (") + String(daysOfTheWeek[now.dayOfTheWeek()]) + String(") ") + " " + String(now.hour(), DEC) + String(':') + String(now.minute(), DEC) + String(':') + String (now.second(), DEC);
        sendMessage(message);
        Serial.println();


// Open the CSV file to write the reading values of each variables (CO2, Temp, RH) (added)//

  myFile = SD.open("logger.csv", FILE_WRITE);

  if (myFile) {
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print('/');
    myFile.print(now.year(), DEC);
    myFile.print(",");
    myFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    myFile.print(",");
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);
    myFile.print(",");
    myFile.print(co2Value,2);
    myFile.print(",");
    myFile.print(tempValue,2);
    myFile.print(",");
    myFile.println(rhValue,2);
       
    // close the file:
    myFile.close();
    Serial.println(F("Data saved on SD Card!!"));
    Serial.println('\n');
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("Error Saving Data. Please, insert the SD Card again and press any key to save the data"));
  }

}

else {
        Serial.println(F("Checksum failed / Communication failure with CO2 Sensor"));
}

delay(2000);

}



