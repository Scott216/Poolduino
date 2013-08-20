/*
 To Do
 Add reset button
 Add temp sensor
 Accelerometer has 10k pullups on the I2C lines, you may not need the external ones you put on.
 On/Off switch
 
 Pool water Level detector with PanStamp
 Want low power consumption, put PanStamp in sleep mode, wake up every 8 seconds to test for water level
 
 Hardware
 panStamp  http://www.panstamp.com/
 MMA8452Q Accelerometer https://www.sparkfun.com/products/10955
 pullup resistors for I2C
 Float switch McMaster 50195K93
 
 MMA8452Q Accelerometer
 Data sheet: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Accelerometers/MMA8452Q.pdf
 You can reduce power consumption by putting it in low power mode.  Also the slower the sampling rate (ODR) the less power.  See tables on pg 7 of the data sheet
 ODR ranges from 800 Hz (default) to 1.56 Hz.  See table 55 on pg 38 for register settings.
 To put in low power mode, set bits 0 and 1 in register CTRL_REG2.  See table 57 on pg 39
 To set ODR to 100 Hz in CTRL_REG1  set bit 3 = 1, bit 4 = 1, bit 5 = 0.
 To set low power mode (MODS=11) in CTRL_REG2 set bits 0 = 1, bit 1 = 1
 
 PanStamp packet structure
 byte 0: Rx ID - ID of Rx panStamp
 byte 1: Tx ID - ID of Tx panStamp
 byte 2: bytes panstamp is sent
 byte 3: Low water detected, low for two minutes. True = Low Water, False = water okay
 byte 4: low water sensor in real time: True = Low Water, False = water okay
 byte 5: Level Lid: false - lid is not level
 byte 6: Battery volts LSB
 byte 7: Battery volts MSB
 
 panStamp Pinout
 
 ANT
 GND         GND
 D8          GND
 D9          D7
 A0          D6
 A1          D5
 A2          D4
 GND         D3
 A3          D1
 A4 SCA      D0
 A5 SCL      GND
 A6          VCC
 A7          RST
 */

// #define PRINT_DEBUG // comment out to turn off printing

#include "Arduino.h"
#include "EEPROM.h"       // panStamp address is saved to EEPROM http://www.arduino.cc/en/Reference/EEPROM
#include "cc1101.h"       // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/cc1101.h
#include "panstamp.h"     // http://code.google.com/p/panstamp/source/browse/trunk/arduino/libraries/panstamp/panstamp.h
#include <Wire.h>         // Used for I2C


// The networkAdress of sender and receiver must be the same
// in the cc1101 documentation this byte is called syncword
// in the SWAP world of the panStamp it is called networkAddress
byte psNetworkAddress =    91;
byte psSenderAddress =      1;  // Address of this panStamp
byte psReceiverAddress =    5;  // Address of panStamp data is sent too
CC1101 cc1101;   // http://code.google.com/p/panstamp/wiki/CC1101class


uint32_t lowWaterTimer;    // Timer used make sure water is low for a period of time
#define LOWWATERTIME 150   // millis doesn't increment when in sleep mode, so you can't use normal milliseconds.  8 seconds = 10mS on millis(), 2 minutes = 150 millis() mS
bool waterIsLow = false;   // This is set if level is low for a few minutes
#define WATERLEVELSENSOR 8 // sensor is connected to D8
#define LOWATER LOW        // Sensor input state then water is low

//Define the registers that we will be accessing on the MMA8452
#define MMA8452_ADDRESS 0x1D  // (decimal 29) 0x1D if SA0 is high, 0x1C if low
#define OUT_X_MSB       0x01
#define XYZ_DATA_CFG    0x0E
#define WHO_AM_I        0x0D
#define CTRL_REG1       0x2A
#define CTRL_REG2       0x2B // for low power mode, set CTRL_REG2 bits 0 and 1 to high  Ref: http://www.freescale.com/files/sensors/doc/app_note/AN4075.pdf
#define GSCALE             2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

// Function Prototypes
bool IsLidFlat();
unsigned int readVcc();
void readAccelData(int *destination);
void initMMA8452();
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
  
  pinMode(WATERLEVELSENSOR, INPUT_PULLUP);  // water level sensor
  
  // Initialize the CC1101 RF Chip
  cc1101.init();
  cc1101.setSyncWord(&psNetworkAddress, false);   // true saves address to EEPROM
  cc1101.setDevAddress(psSenderAddress, false);   // true saves address to EEPROM
  
  initMMA8452(); // Intialize the MMA8452Q Accelerometer
  
  lowWaterTimer = millis() + LOWWATERTIME;
  
#ifdef PRINT_DEBUG
  Serial.println("Accelerometer enabled");
  delay(200); // need a delay after printing to prevent garbled data
#endif // PRINT_DEBUG
  
} // setup()

void loop()
{
 
  static uint16_t batteryOld; // used previous battery reading if current one is invalid
  
  if( digitalRead(WATERLEVELSENSOR) == LOWATER && IsLidFlat() == true)
  { // Low water detected
    
    // See if water level has been low for 2 minutes or more
    if((long) (millis() - lowWaterTimer) > 0)
    { waterIsLow = true; }
    else
    { waterIsLow = false; }
  }
  else
  { // Water level is okay, reset low water timer
    lowWaterTimer = millis() + LOWWATERTIME;
    waterIsLow = false;
  }
#ifdef PRINT_DEBUG
  Serial.println("Transmitting data");
  delay(200); // need a delay after printing to prevent garbled data
#endif
  CCPACKET data;
  data.length = 8;                     // # bytes that make up packet to transmit
  data.data[0] = psReceiverAddress;    // Address of panStamp Receiver we are sending too. THIS IS REQUIRED BY THE CC1101 LIBRARY
  data.data[1] = psSenderAddress;      // Address of this panStamp Tx
  data.data[2] = data.length;          // bytes panstamp is sending
  data.data[3] = waterIsLow;           // Water level status true/false
  if( digitalRead(WATERLEVELSENSOR) == LOWATER ) // Live level sensor input.  False = level okay, true = level is low
  { data.data[4] = true; }
  else
  { data.data[4] = false; }
  data.data[5] = IsLidFlat();          // IsLidFlat
  uint16_t battery = readVcc();        // Read battery voltage
  if(battery < 2000)  // 2 volts
  { battery = batteryOld; }  // use old value
  else
  { batteryOld = battery; }  // update old value with current reading
  data.data[6] = battery >> 8 & 0xff;  // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
  data.data[7] = battery & 0xff;       // Low byte, just mask off the upper 8 bits
  
  cc1101.sendData(data); // Send out data
  
  panstamp.sleepWd(WDTO_8S);  // Sleep for 8 seconds. millis() doesn't increment while sleeping
  
}  // end loop()


// Read the accelerometer and determine if the skimmer lid is flat or not
// If not it means someone has removed the lid and the level detector should be ignored
bool IsLidFlat()
{
  
  // read accelerometer
  int accelCount[3];  // Stores the 12-bit signed value
  readAccelData(accelCount);  // Read the x/y/z adc values
  
  if (accelCount[0] > -100 && accelCount[0] < 100 &&
      accelCount[1] > -100 && accelCount[1] < 100 &&
      accelCount[2] > 1000 && accelCount[2] < 1100 )
  { return true; }
  else
  { return false; }
}


// Read battery volts, returned value is in millivolts
unsigned int readVcc()
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
  
} // readVcc()


void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here
  
  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array
  
  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer
    
    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }
    
    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {
    //    Serial.println("MMA8452Q is online...");
  }
  else
  {
    //    Serial.print("Could not connect to MMA8452Q: 0x");
    //    Serial.println(c, HEX);
    //    while(1) ; // Loop forever if communication doesn't happen
  }
  
  MMA8452Standby();  // Must be in standby to change registers
  MMA8452LowPower(); // Setup low power.  Added by SRG
  
  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);
  
  //The default data rate is 800Hz and we don't modify it in this example code
  
  MMA8452Active();  // Set to active to start reading
}


// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}


// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

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
  
}


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
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active
  
  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default
  
  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}
