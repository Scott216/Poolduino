/*
Pool water level sensor - Rx
Use PanStamp to recieves data from water level sensor
Acts as I2C slave and sends data to main Arduino pool controller

 
PanStamp packet structure
byte 0:     Rx ID - ID of Rx panStamp
byte 1:     Tx ID - ID of Tx panStamp
byte 2:     Bytes in panstamp packet
byte 3:     Water Level 2 Min: 0 = level ok, 1 = level low
byte 4:     Water Level LIVE:  0 = level ok, 1 = level low
byte 5:     Is lid flat: true/false
byte 6,7:   Accelerometer x-axis value
byte 8,9:   Accelerometer y-axis value
byte 10,11: Accelerometer z-axis value
byte 12,13: Battery volts
byte 14:    Reserved for water leaking inside sensor
byte 15:    Checksum

I2C Packet structure
byte 0:     I2C Slave address (this panStamp is the slave)
byte 1:     TX panStamp Status: 255 = offline, 1 = online
byte 2:     Water Level 2 Min: 0 = level ok, 1 = level low, 2 = sensor offline
byte 3:     Water Level LIVE:  0 = level ok, 1 = level low
byte 4:     Is lid flat: true/false
byte 5,6:   Accelerometer x-axis value
byte 7,9:   Accelerometer y-axis value
byte 9,10:  Accelerometer z-axis value
byte 11,12: Battery voltage
byte 13:    Reserved for water leaking inside sensor
byte 14:    Checksum
 
 
Change log
v1.10 08/17/14  Added checksum. Formatting. Change real time level so 0 = level okay
v1.11 08/18/14 - changed panStampOk from 0 to 1
*/

// #define PRINT_DEBUG // comment out to turn off serial printing

#include "EEPROM.h"       // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"       // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h"     // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
#include <Wire.h>         // Used for I2C

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// The networkAdress of panStamp sender and receiver must be the same
byte psNetworkAdress =         91;  // Network address for all pool panStamps 
byte psReceiverAddress =        5;  // Device address of this panStamp
const byte addrSlaveI2C =      21;  // I2C Slave address of this device
const byte addrLevelSensor =    1;  // panStamp device address for low water sensor
const byte panStampOffline =  255;  // Send this to I2C master in the panStamp Rx address to indicate panStamp is offline
const byte panStampOK      =    1;  // panStamp is successfully transmitting data
uint32_t psTxTimer = 0; 
#define PSTIMEOUT 120000      // 2 minute timeout for panStamps.  If no connections in 2 minutes, tell master that panStamp is offline

#define I2C_PACKET_LEN 16 // bytes in I2C Packet
byte I2C_Packet[I2C_PACKET_LEN];   // Array to hold data sent over I2C to main Arduino

CCPACKET packet;  // panStamp data http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/ccpacket.h


// The connection to the hardware chip CC1101 the RF Chip
CC1101 cc1101;  // http://code.google.com/p/panstamp/wiki/CC1101class


// flag indicates wireless packet has been received
volatile boolean psPacketAvailable = false;        

// Function Prototypes
void cc1101signalsInterrupt(void);
void wireRequestEvent();

//========================================================================================================================================
// Handle interrupt from CC1101 (INT 0)
//========================================================================================================================================
void cc1101signalsInterrupt(void)
{
  // set the flag that a package is available
  psPacketAvailable = true;
} // end cc1101signalsInterrupt()


//========================================================================================================================================
//========================================================================================================================================
void setup()
{
  Serial.begin(9600);

  Wire.begin(addrSlaveI2C);    // Initiate the Wire library and join the I2C bus 
  Wire.onRequest(wireRequestEvent); // Register a function to be called when a master requests data from this slave device. 
 
  cc1101.init(); // initialize the RF Chip in panStamp
  cc1101.setSyncWord(&psNetworkAdress, false);  // Set network address (pointer), false parameter tells function not to save to EEPROM
  // This receiverAddress needs to match the receiverAddress in the Tx panStamp
  cc1101.setDevAddress(psReceiverAddress, false);  // false parameter tells function not to save to EEPROM
  cc1101.enableAddressCheck(); // you can skip this line, because the default is to have the address check enabled
  cc1101.setRxState(); // Set this panStamp to be a receiver
  attachInterrupt(0, cc1101signalsInterrupt, FALLING); // Enable wireless reception interrupt
  
  #ifdef PRINT_DEBUG 
  // Print device setup info
    Serial.print(F("Radio Frequency = "));
    if(cc1101.carrierFreq == CFREQ_868)
    {Serial.println(F("868 Mhz"));}
    else
    {Serial.println(F("915 Mhz"));}
    Serial.print(F("Channel = "));
    Serial.println(cc1101.channel);
    Serial.print(F("Network address = "));
    Serial.println(cc1101.syncWord[0]);
    Serial.print(F("Device address =  "));
    Serial.println(cc1101.devAddress);
  #endif
  
  psTxTimer = millis() + PSTIMEOUT;  // Initialize Tx timeout timer

}  // end setup()

//========================================================================================================================================
//========================================================================================================================================
void loop()
{
  
  // Get data from water level sensor panStamp
  if(psPacketAvailable)
  {
    // clear the flag
    psPacketAvailable = false;
    
    // Disable wireless reception interrupt so this code finishes executing without inturruption
    detachInterrupt(0);
    
    if(cc1101.receiveData(&packet) > 0)
    {
      if (packet.crc_ok && packet.length > 1)
      {  
        psTxTimer = millis() + PSTIMEOUT;  // reset timeout timer
        
      #ifdef PRINT_DEBUG
        // print received data
        char buf[40];
        sprintf(buf, "Received %d bytes from panStamp %d", packet.length, packet.data[1]);
        Serial.println(buf);
        Serial.print(F("Rx ID: "));
        Serial.print(packet.data[0]);
        Serial.print(F("   Tx ID: "));
        Serial.println(packet.data[1]);
        Serial.print(F("Low Water Status: "));
        Serial.print(packet.data[3]);
        Serial.print(F("   Low Water Real Time: "));
        Serial.println(packet.data[4]);
        if (packet.data[5] == true)
        { Serial.println(F("Lid is level")); }
        else
        { Serial.println(F("Lid is not level")); }
        Serial.print(F("Battery LSB: "));
        Serial.print(packet.data[11]);
        Serial.print(F("   Battery MSB: "));
        Serial.println(packet.data[12]);
        Serial.println();
      #endif
       
        // Copy data from panStamp packet to I2C packet array
        I2C_Packet[0] = addrSlaveI2C;
        I2C_Packet[1] = panStampOK;
        I2C_Packet[2] = packet.data[3];    // Low water level, Water level sensor 0 = level ok, 1 = level is low, 2 = sensor offline
        I2C_Packet[3] = packet.data[4];    // Real time Low water level: true = low water, false = water ok
        I2C_Packet[4] = packet.data[5];    // Is Lid Flat: true/false
        I2C_Packet[5] = packet.data[6];    // Accelerometer x-axis
        I2C_Packet[6] = packet.data[7];    // Accelerometer x-axis
        I2C_Packet[7] = packet.data[8];    // Accelerometer y-axis
        I2C_Packet[8] = packet.data[9];    // Accelerometer y-axis
        I2C_Packet[9] = packet.data[10];   // Accelerometer x-axis
        I2C_Packet[10] = packet.data[11];  // Accelerometer x-axis
        I2C_Packet[11] = packet.data[12];  // Battery voltage
        I2C_Packet[12] = packet.data[13];  // Battery voltage
        I2C_Packet[13] = packet.data[14];  // Reserved for leak detected in electronics
        // checksum
        I2C_Packet[14] = 0;
        for (byte cs = 0; cs < 14; cs++)
        { I2C_Packet[14] += I2C_Packet[cs]; }
                                 
      } // packet is okay
    }  // got packet
    
    // Enable panStamp wireless reception interrupt
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  }

  // If we haven't received data from transmitter in 2 minutes, Tx is considered offline
  if ((long)(millis() - psTxTimer) > 0 )    
  { 
    // panStamp is offline, set I2C packet appropriately
    memset(I2C_Packet,0,sizeof(I2C_Packet));  // clear array
    I2C_Packet[0] = addrSlaveI2C;
    I2C_Packet[1] = panStampOffline;
    I2C_Packet[2] = 2; // 2 = offline
    // checksum
    for (byte cs = 0; cs < 3; cs++)
    { I2C_Packet[14] += I2C_Packet[cs]; }
                                 
  }  // end psPacketAvailable
  
  
} // end loop()


//========================================================================================================================================
// function that executes whenever data is requested by master
// this function is registered as an event in setup()
//========================================================================================================================================
void wireRequestEvent()
{
  // Send data to I2C master
  Wire.write(I2C_Packet, I2C_PACKET_LEN);
  
} // end wireRequestEvent()
