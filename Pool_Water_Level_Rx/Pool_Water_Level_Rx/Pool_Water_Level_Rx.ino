

/* 
PanStamp Level detector Receiver
Recieves data from sensorl. 
Acts as I2C slave to send data to main Arduino

PanStamp packet structure
byte 0: Rx ID - ID of Rx panStamp 
byte 1: Tx ID - ID of Tx panStamp
byte 2: bytes panstamp is sent
byte 3: Low water detected: true/false
byte 4: low water sensor - real time, True = low water, False - level okay.  Ignores if lid is level
byte 5: Level Lid: false - lid is not level
byte 6: Battery volts LSB
byte 7: Battery volts MSB

I2C Packet structure
byte 0: I2C Slave address
byte 1: TX panStamp Status: 255 = offline, 0 = online
byte 2: water level sensor 0 - level ok, 1 - level is low, 2 - sensor offline
byte 3: Live low water sensor
byte 4: Level Lid: false - lid is not level
byte 5: battery voltage LSB
byte 6: battery voltage MSB

*/


#include "Arduino.h"
#include "EEPROM.h"       // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"       // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h"     // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
#include <Wire.h>         // Used for I2C


// #define PRINT_DEBUG // comment out to turn off serail printing

// The networkAdress of panStamp sender and receiver must be the same
byte psNetworkAdress =         91;  // Network address for all pool panStamps 
byte psReceiverAddress =        5;  // Device address of this panStamp
const byte addrSlaveI2C =      21;  // I2C Slave address of this device
const byte addrLevelSensor =    1;  // panStamp device address for low water sensor
const byte panStampOffline =  255;  // Send this to I2C master in the panStamp Rx address to indicate panStamp is offline
const byte panStampOK      =    0;  // panStamp is successfully transmitting data
uint32_t psTxTimer = 0; 
#define PSTIMEOUT 120000      // 2 minute timeout for panStamps.  If no connections in 2 minutes, tell master that panStamp is offline

#define I2C_PACKET_LEN 7 // bytes in I2C Packet
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
        Serial.print("Rx ID: ");
        Serial.print(packet.data[0]);
        Serial.print("   Tx ID: ");
        Serial.println(packet.data[1]);
        Serial.print("Low Water Status: ");
        Serial.print(packet.data[3]);
        Serial.print("   Low Water Real Time: ");
        Serial.println(packet.data[4]);
        if (packet.data[5] == true)
        { Serial.println("Lid is level"); }
        else
        { Serial.println("Lid is not level"); }
        Serial.print("Battery LSB: ");
        Serial.print(packet.data[6]);
        Serial.print("   Battery MSB: ");
        Serial.println(packet.data[7]);
        Serial.println();
      #endif
       
        // Copy data from panStamp packet to I2C packet array
        I2C_Packet[1] = panStampOK;
        I2C_Packet[2] = packet.data[3];   // Low water level, low for 2 mintues. true - low water, false - water ok
        I2C_Packet[3] = packet.data[4];   // Low water level, real time. true - low water, false - water ok
        I2C_Packet[4] = packet.data[5];   // Is Lid Flat: true/false
        I2C_Packet[5] = packet.data[6];   // Battery voltage
        I2C_Packet[6] = packet.data[7];   // Battery voltage

      } // packet is okay
    }  // got packet
    
    // Enable panStamp wireless reception interrupt
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  }

  // If we haven't received data from transmitter in 2 minutes, Tx is considered offline
  if ((long)(millis() - psTxTimer) > 0 )    
  { 
    // panStamp is offline, set I2C packet appropriately
    I2C_Packet[1] = panStampOffline; 
    I2C_Packet[2] = 2; // 2 = offline
    I2C_Packet[3] = 0; 
    I2C_Packet[4] = 0; 
    I2C_Packet[5] = 0; 
    I2C_Packet[6] = 0; 
  }  // end psPacketAvailable
  
  
} // end loop()


//========================================================================================================================================
// function that executes whenever data is requested by master
// this function is registered as an event in setup()
//========================================================================================================================================
void wireRequestEvent()
{
  // Send data to I2C master
  I2C_Packet[0] = addrSlaveI2C;
  Wire.write(I2C_Packet, I2C_PACKET_LEN); 
  
} // end wireRequestEvent()
