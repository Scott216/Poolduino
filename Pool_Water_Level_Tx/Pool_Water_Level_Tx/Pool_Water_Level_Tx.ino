/*
Monitors pool level and wirelessly sends status to 



To Do
Accelerometer has 10k pullups on the I2C lines, you may not need the external ones you put on.
On/Off switch
Add checksum to wireless data

Note: had a problem one time where water was low, but it wasn't setting the low level variable.  Sketch was reporting lid was not flat when it was for some reason.

Pool water Level detector with PanStamp
Want low power consumption, put PanStamp in sleep mode, wake up every 8 seconds to test for water level

Hardware
panStamp  http://www.panstamp.com/
MMA8452Q Accelerometer http://www.sparkfun.com/products/10955
Pullup resistors for I2C
Float switch McMaster 50195K93

MMA8452Q Accelerometer
Data sheet: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Accelerometers/MMA8452Q.pdf
You can reduce power consumption by putting it in low power mode.  Also the slower the sampling rate (ODR) the less power.  See tables on pg 7 of the data sheet
ODR ranges from 800 Hz (default) to 1.56 Hz.  See table 55 on pg 38 for register settings.
To put in low power mode, set bits 0 and 1 in register CTRL_REG2.  See table 57 on pg 39
To set ODR to 100 Hz in CTRL_REG1  set bit 3 = 1, bit 4 = 1, bit 5 = 0.
To set low power mode (MODS=11) in CTRL_REG2 set bits 0 = 1, bit 1 = 1

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

 
Change log
v1.10 08/17/14  Formatting, changed some #define to const. Added checksum.  Added sendData() function.  Changed packet byte 4 - live level to match 2 minute level
 
*/

// #define PRINT_DEBUG // comment out to turn off printing

#include "EEPROM.h"       // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"       // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h"     // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
#include <Wire.h>         // Used for I2C

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// The networkAdress of sender and receiver must be the same
// in the cc1101 documentation this byte is called syncword
// in the SWAP world of the panStamp it is called networkAddress
// Can't use constants for these
byte psNetworkAddress =    91;
byte psSenderAddress =      1;  // Address of this panStamp
byte psReceiverAddress =    5;  // Address of panStamp data is sent too
CC1101 cc1101;   // http://code.google.com/p/panstamp/wiki/CC1101class


const byte LOWWATERTIME =         150; // millis doesn't increment when in sleep mode, so you can't use normal milliseconds.  8 seconds = 10mS on millis(), 2 minutes = 150 millis() mS
const byte WATERLEVELSENSOR_PIN =   8; // level sensor is connected to pin D8
#define LOW_WATER LOW                  // Sensor input state is LOW when water level is low

//Define the registers that we will be accessing on the MMA8452
#define MMA8452_ADDRESS 0x1D  // (decimal 29) 0x1D if SA0 is high, 0x1C if low
#define OUT_X_MSB       0x01
#define XYZ_DATA_CFG    0x0E
#define WHO_AM_I        0x0D
#define CTRL_REG1       0x2A
#define CTRL_REG2       0x2B // for low power mode, set CTRL_REG2 bits 0 and 1 to high  Ref: http://www.freescale.com/files/sensors/doc/app_note/AN4075.pdf
#define GSCALE             2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.


bool isAccelOnline = false; // flag to indicate if accelerometer is online
int accelCount[3];          // Stores the 12-bit signed value


// Function Prototypes
void sendData(bool waterIsLow);
bool IsLidFlat();
uint16_t  readVcc();
void readAccelData(int *destination);
bool initMMA8452();
void MMA8452Standby();
void MMA8452Active();
void MMA8452LowPower();
void readRegisters(byte addressToRead, int bytesToRead, byte * dest);
byte readRegister(byte addressToRead);
void writeRegister(byte addressToWrite, byte dataToWrite);


void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  pinMode(WATERLEVELSENSOR_PIN, INPUT_PULLUP);  // water level sensor
  
  // Initialize the CC1101 RF Chip
  cc1101.init();
  cc1101.setSyncWord(&psNetworkAddress, false);   // true saves address to EEPROM
  cc1101.setDevAddress(psSenderAddress, false);   // true saves address to EEPROM
  
  isAccelOnline = initMMA8452(); // Intialize the MMA8452Q Accelerometer
  
  #ifdef PRINT_DEBUG
    Serial.println(F("Accelerometer enabled"));
    delay(200); // need a delay after printing to prevent garbled data
  #endif // PRINT_DEBUG
  
} // end setup()


void loop()
{
  static uint32_t lowWaterTimer = millis() + LOWWATERTIME;  // Timer used make sure water is low for a period of time

  // Check for low water level
  bool waterIsLow = false;   // This is set if level is low for a few minutes
  if( digitalRead(WATERLEVELSENSOR_PIN) == LOW_WATER && IsLidFlat() )
  {
    // See if water level has been low for 2 minutes or more
    if( (long)(millis() - lowWaterTimer) > 0 )
    { waterIsLow = true; }
    else
    { waterIsLow = false; }
  }
  else
  { // Water level is okay or lid is not flat, reset low water timer
    lowWaterTimer = millis() + LOWWATERTIME;
    waterIsLow = false;
  }
  
  #ifdef PRINT_DEBUG
    Serial.println("Transmitting data");
    delay(200); // Need a delay after printing to prevent garbled data
  #endif
  
  sendData(waterIsLow);
  
  panstamp.sleepWd(WDTO_8S);  // Sleep for 8 seconds. millis() doesn't increment while sleeping
  
}  // end loop()


void sendData(bool waterIsLow)
{
  CCPACKET data;
  data.length = 16;                    // # bytes that make up packet to transmit
  data.data[0] = psReceiverAddress;    // Address of panStamp Receiver we are sending too. THIS IS REQUIRED BY THE CC1101 LIBRARY
  data.data[1] = psSenderAddress;      // Address of this panStamp Tx
  data.data[2] = data.length;          // bytes panstamp is sending
  data.data[3] = waterIsLow;           // Water level status true/false
  if( digitalRead(WATERLEVELSENSOR_PIN) == LOW_WATER ) // Live level sensor input
  { data.data[4] = 1; }  // real time water level low
  else
  { data.data[4] = 0; }  // real time water level okay
  data.data[5] = IsLidFlat();
  data.data[6] = accelCount[0] >> 8 & 0xff;  // x-axis accelerometer value
  data.data[7] = accelCount[0] & 0xff;
  data.data[8] = accelCount[1] >> 8 & 0xff;  // y-axis accelerometer value
  data.data[9] = accelCount[1] & 0xff;
  data.data[10] = accelCount[2] >> 8 & 0xff; // z-axis accelerometer value
  data.data[11] = accelCount[2] & 0xff;
  
  static uint16_t batteryOld;    // Use previous battery reading if current one is invalid (less then 2 volts)
  uint16_t battery = readVcc();
  if(battery < 2000)  // 2 volts
  { battery = batteryOld; }
  else
  { batteryOld = battery; }
  data.data[12] = battery >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
  data.data[13] = battery & 0xff;       // Low byte, just mask off the upper 8 bits
  data.data[14] = false;                // Reserved for water leaking inside electronics enclosure
  
  // Calculate checksum
  data.data[15] = 0;
  for (byte cs = 0; cs < data.length-1; cs++)
  { data.data[15] += data.data[cs]; }
  
  cc1101.sendData(data); // Transmit data
  
} // end sendData()


// Read the accelerometer and determine if the skimmer lid is flat or not
// If not it means someone has removed the lid and the level detector should be ignored
bool IsLidFlat()
{
  readAccelData(accelCount);
  
  if (accelCount[0] > -100 && accelCount[0] < 100 &&
      accelCount[1] > -100 && accelCount[1] < 100 &&
      accelCount[2] > 1000 && accelCount[2] < 1100 )
  { return true; }
  else
  { return false; }
}  // end IsLidFlat()


// Read battery volts, returned value is in millivolts
uint16_t readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  
  long result = (high<<8) | low;
  
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (unsigned int) result; // Vcc in millivolts
  
} // end readVcc()


void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  
  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array
  
  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  // Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; // The registers are left align, here we right align the 12-bit integer
    
    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }
    
    destination[i] = gCount; // Record this gCount into the 3 int array
  }
}  // end readAccelData()


// Initialize the MMA8452 registers
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
bool initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {
    // MMA8452Q is online
    MMA8452Standby();  // Must be in standby to change registers
    MMA8452LowPower(); // Setup low power.  Added by SRG
    
    // Set up the full scale range to 2, 4, or 8g.
    byte fsr = GSCALE;
    if(fsr > 8) fsr = 8; //Easy error check
    fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
    writeRegister(XYZ_DATA_CFG, fsr);
    
    //The default data rate is 800Hz and we don't modify it in this example code
    
    MMA8452Active();  // Set to active to start reading
    return true;
  }
  else
  {
    // Could not connect to MMA8452Q
    return false;
  }
  
}  // end initMMA8452()


// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}  // end MMA8452Standby()


// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}  // end MMA8452Active()


// Put MMA8452 in low power mode (MODS = 11) and ODR = 100Hz.  Reduces power consumption to 24uA
// Added by SRG
void MMA8452LowPower()
{
  // Set ODS Frequency to 100 Hz (default is 800 Hz).
  byte c = readRegister(CTRL_REG1);
  c |= 1 << 3;
  c |= 1 << 4;
  c &= ~(1 << 5);
  writeRegister(CTRL_REG1, c );
  
  // put in low power mode
  c = readRegister(CTRL_REG2);
  writeRegister(CTRL_REG2, c | 0x03);  // set bit's 0 and 1
  
}  // end MMA8452LowPower()


// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active
  
  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default
  
  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect
  
  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();
}  // end readRegisters()


// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); // endTransmission but keep the connection active
  
  Wire.requestFrom(MMA8452_ADDRESS, 1); // Ask for 1 byte, once done, bus is released by default
  
  while(!Wire.available()) ; // Wait for the data to come back
  return Wire.read(); // Return this one byte
}  // end readRegister()


// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}  // end writeRegister()

