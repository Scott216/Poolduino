/*
Pool water level sensor - Rx
Use PanStamp to recieves data from water level sensor
Acts as I2C slave and sends data to main Arduino pool controller


To do:
Find out if interrupts should be disabled in radioSignalInterrupt() so the wire wireRequestEvent interrupt doesn't interfere with panstamp interrupt
 
 
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
byte 7,8:   Accelerometer y-axis value
byte 9,10:  Accelerometer z-axis value
byte 11,12: Battery voltage
byte 13:    Reserved for water leaking inside sensor
byte 14:    Checksum
 
 
Change log
08/17/14 v1.10 - Added checksum. Formatting. Change real time level so 0 = level okay
08/18/14 v1.11 - changed panStampOk from 0 to 1.
08/18/14 v1.12 - Changed I2C_PACKET_LEN from 16 to 15, renamed some constants
06/15/15 v1.13 - Changed panstamp timeout (PSTIMEOUT) from 2 minutes to 10 minutes.  Added VERSION constant and prints it in setup()
                 Updated sketch to be compatible with panStamp V2 API
*/

#define VERSION "v1.13"
// #define PRINT_DEBUG // comment out to turn off serial printing

#include <HardwareSerial.h>
#include "EEPROM.h"       // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
//#include "cc1101.h"       // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
//#include "panstamp.h"     // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
#include <Wire.h>         // Used for I2C

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// The networkAdress of panStamp sender and receiver must be the same
const byte g_RF_Channel =        0;   // panStamp channel
byte g_psNetworkAddress[] = {91, 0};  // panStamp network address, aka SyncWord
byte g_psReceiverAddress =       5;   // Device address of this panStamp
const byte ADDR_SLAVE_I2C =     21;   // I2C Slave address of this device
const byte PANSTAMP_OFFLINE =  255;   // Send this to I2C master in the panStamp Rx address to indicate panStamp is offline
const byte PANSTAMP_OK =         1;   // panStamp is successfully transmitting data
const byte I2C_PACKET_LEN =     15;   // bytes in I2C Packet
      byte I2C_Packet[I2C_PACKET_LEN];      // Array to hold data sent over I2C to main Arduino
const uint32_t PSTIMEOUT = 600000;  // Timeout for panStamps.  If no connections in 2 minutes, tell master that panStamp is offline
uint32_t g_psTxTimer =  PSTIMEOUT;  // Initialize Tx timeout timer

// flag indicates wireless packet has been received
volatile boolean g_psPacketAvail = false;        

// Function Prototypes
void radioSignalInterrupt(CCPACKET *psPacket);
void wireRequestEvent();

//========================================================================================================================================
// Handle interrupt from CC1101 (INT 0)
//========================================================================================================================================
void radioSignalInterrupt(CCPACKET *psPacket)
{
  if( psPacket->crc_ok && psPacket->length > 1 )
  { 
    g_psPacketAvail = true;  // set the flag that a package is available
    g_psTxTimer = millis() + PSTIMEOUT;  // reset timeout timer
    // Copy data from panStamp packet to I2C packet array
    I2C_Packet[0] =  ADDR_SLAVE_I2C;
    I2C_Packet[1] =  PANSTAMP_OK;
    I2C_Packet[2] =  psPacket->data[3];    // Low water level, Water level sensor 0 = level ok, 1 = level is low, 2 = sensor offline
    I2C_Packet[3] =  psPacket->data[4];    // Real time Low water level: true = low water, false = water ok
    I2C_Packet[4] =  psPacket->data[5];    // Is Lid Flat: true/false
    I2C_Packet[5] =  psPacket->data[6];    // Accelerometer x-axis
    I2C_Packet[6] =  psPacket->data[7];    // Accelerometer x-axis
    I2C_Packet[7] =  psPacket->data[8];    // Accelerometer y-axis
    I2C_Packet[8] =  psPacket->data[9];    // Accelerometer y-axis
    I2C_Packet[9] =  psPacket->data[10];   // Accelerometer x-axis
    I2C_Packet[10] = psPacket->data[11];  // Accelerometer x-axis
    I2C_Packet[11] = psPacket->data[12];  // Battery voltage
    I2C_Packet[12] = psPacket->data[13];  // Battery voltage
    I2C_Packet[13] = psPacket->data[14];  // Reserved for leak detected in electronics
    // checksum
    I2C_Packet[I2C_PACKET_LEN - 1] = 0;
    for ( byte cs = 0; cs < I2C_PACKET_LEN - 1; cs++ )
    { I2C_Packet[I2C_PACKET_LEN - 1] += I2C_Packet[cs]; }
  } 
} // end radioSignalInterrupt()


//========================================================================================================================================
//========================================================================================================================================
void setup()
{
  Serial.begin(9600);

  Wire.begin(ADDR_SLAVE_I2C);    // Initiate the Wire library and join the I2C bus
  Wire.onRequest(wireRequestEvent); // Register a function to be called when a master requests data from this slave device. 

  // Setup the panStamp radio
  panstamp.radio.setChannel(g_RF_Channel);
  panstamp.radio.setSyncWord(g_psNetworkAddress);  // Set network address, pointer to address
  panstamp.radio.setDevAddress(g_psReceiverAddress);
  panstamp.setPacketRxCallback(radioSignalInterrupt);  // Declare RF callback function

  g_psTxTimer = millis() + PSTIMEOUT;  // Initialize Tx timeout timer

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
    Serial.print("Sketch Version ");
    Serial.println(VERSION);
  #endif
  

}  // end setup()

//========================================================================================================================================
//========================================================================================================================================
void loop()
{
  
  if(g_psPacketAvail)
  {
    panstamp.rxOff();  // turn panStamp off while processing data
    g_psPacketAvail = false; // Reset panStamp paccket avail flag
    panstamp.rxOn(); // Finished prcessing, turn panStamp back on 
  }  

  // If we haven't received data from transmitter in 10 minutes, Tx is considered offline
  if ((long)(millis() - g_psTxTimer) > 0 )    
  { 
    // panStamp is offline, set I2C packet appropriately
    memset(I2C_Packet, 0, sizeof(I2C_Packet));  // clear array
    I2C_Packet[0] = ADDR_SLAVE_I2C;
    I2C_Packet[1] = PANSTAMP_OFFLINE;
    I2C_Packet[2] = 2; // water level offline
    // checksum
    for (byte cs = 0; cs < 3; cs++)
    { I2C_Packet[I2C_PACKET_LEN - 1] += I2C_Packet[cs]; }
  }  
  
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
