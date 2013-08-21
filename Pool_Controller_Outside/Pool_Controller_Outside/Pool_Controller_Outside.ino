#include "Arduino.h"
#include "LocalLibrary.h"


#include <Wire.h>       // http://www.arduino.cc/en/Reference/Wire
#include <RTClib.h>     // http://github.com/adafruit/RTClib
//#include <max6675.h>    // Thermocouple Amplifier Library http://github.com/adafruit/MAX6675-library
#include <XBee.h>       //http://code.google.com/p/xbee-arduino/     Modified per http://arduino.cc/forum/index.php/topic,111354.0.html
// #include <SoftwareSerial.h>
#include <Button.h>     // For pushbutton http://github.com/carlynorama/Arduino-Library-Button
#include <OneWire.h>    // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library

#define PRINT_DEBUG                   // Comment out when done debugging


//=== Digital I/O ===

//=== Analog Inputs ===
#define PRESSURE1_PIN        0   // Pressure before filter
#define PRESSURE2_PIN        1   // Pressure after filter
#define WATER_FILL_PRESSURE  5   // Pressure Transducer connected to water fill line.  Transducer is analog output, but sketch is reading as digital on/off
#define PUMP_AMPS_PIN        3   // Pump amps input 20 Amp CT
#define WATER_FILL_PB       A4   // Push-button for water fill, use as digital input - fills water 15 minutes each time it's pressed
                                 // A5 unused
                                 //=== Digital I/0 ===
#define XBEE_RX              0   // Communicate with Xbee
#define XBEE_TX              1   // Communicate with Xbee
                                 // Leonardo Pins 2 and 3 are used by I2C for the RTC
                                 // D4 unused
#define ONE_WIRE_BUS         5   // OneWire temperture sensor bus
#define TC_AMP_CS_P          6   // MAX6675 CS for Pump housing temperature
#define TC_AMP_DO            7   // MAX6675 DO
#define TC_AMP_CLK           8   // MAX6675 CLK  - Thermocouple dooesn't when using PIN 3 a CLK
#define PUMP_OUTPUT          9   // Turns pump on/off
#define WATER_FILL_PB_LED   10   // LED on Water Fill Pushbutton
#define WATER_OUTPUT        11   // Adds water to pool
#define PUMP_INPUT_AUTO     12   // Pump is on schedule
#define PUMP_INPUT_MAN      13   // Pump manual override
#define PRE_HEAT_TEMP        0   // Pre-heater temperature
#define POST_HEAT_TEMP       1   // Post-heater temperature
#define PUMP_TEMP            2   // Pump temperature
#define PRE_FILTER_PRESSURE  0   // Pre-filter water pressure
#define POST_FILTER_PRESSURE 1   // Post-filter water pressure
#define WATER_FILL_PRESSURE  2   // Water fill pressure


// Adjustment to temperature probes
#define CALIBRATE_TEMP1  -3.0
#define CALIBRATE_TEMP2  -3.5

#define SENSOR_READ_INTERVAL    250   // Read sensor every 250mS
#define TX_INTERVAL            2000   // Send data to house every 2 seconds
#define ACK_RESPONSE_WAIT_TIME  250   // Time (ms) program waits for an ACK response from other XBee. Normal response is about 15mS
#define PUMP_ON_TIME            7.5   // 7:30 AM
#define PUMP_OFF_TIME          19.0   // 7:00 PM
#define PUMP_SWITCH_ON          LOW   // Pump switch is ON when input is LOW, otherwise it's high from the pull-up resistor
#define XBEE_MY_ADDR_RX       0x250   // The MY address of the Rx XBee
#define BAUD_RATE              9600   // Baud for bith Xbee and serial monitor
#define WATER_FILL_BP_MIN    908000   // 15 minutes added to water fill timer
#define WATER_FILL_PRESS_THRESH 240   // SRG Water fill pressure threshold.  ADC value that input should exceeed to assume there garden hose is pressurized
                                      // With 0-30PSI transducer, analog input = 1023 at with valve off, 290 with valve open, 190 with hose diconnected
#define MAX_WATER_FILL_MINUTES  120   // Maximum number of minutes water can be added each day.  Reset at 11 PM

// Status code to send to inside Arduino
static byte poolStatus;
const byte  statusPumpOff =              0;
const byte  statusPumpOn =               1;
const byte  statusPumpSwitchOff =        2;
const byte  statusAddingWater =          3;
const byte  statusEmergencyLoPresFluct = 4;  // low pressure because of fluctuations
const byte  statusEmergencyLoPresCont =  5;  // continious low pressure for 5 mintutes
const byte  statusEmergencyHiAmps =      6;
const byte  statusEmergencyHiPumpTemp =  7;
const byte  statusShutdnFromWeb =        8;

byte LowLevelSensor = 0;    // Water level ok = 0, water level low = 1, offline = 2
uint16_t LevelSensorBatt = 0;  // Battery voltage for level sensor

// Initialize Real Time Clock
RTC_DS1307 RTC;

#define addrSlaveI2C    21  // I2C Slave address of RX panStamp
#define I2C_PACKET_SIZE  7  // I2C Packet size


// Initialize OneWire temp sensors
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature waterTempSensors(&oneWire);
static uint8_t tempSensors[3][8] =
{
  { 0x10, 0xD0, 0x8E, 0x6A, 0x02, 0x08, 0x00, 0xE4 },  // pre heater temp sensor
  { 0x10, 0xCB, 0x95, 0x6A, 0x02, 0x08, 0x00, 0x4B },  // post heater temp sensor
  { 0x10, 0x07, 0x91, 0x6A, 0x02, 0x08, 0x00, 0xB6 }   // Pump temp sensor
};


// Intialize Xbee object
XBee xbee = XBee();
#define NUM_DATA_PTS   16  // Number of integers (data points) to upload. Can't exceed 100 bytes or 50 integers unless you change MAX_FRAME_DATA_SIZE in XBee.h
/* XBee packet structure
 0 Temp Pre Heater
 1 Temp Post Heater
 2 Temp Pump houseing
 3 Pump Amps
 4 Pressure pre-filter
 5 Pressure post filter
 6 Pressure Water Fill
 7 Low pressure counter
 8 Controller Status Number
 9 Minutes of water added today
 10 Water fill countdown
 11 Pool time
 12 Water level sensor battery voltage
 13 Low Water Level - calculated
 14 Sensor Input Status Byte
 15 Dicrete I/O status byte
 
 Sensors working ok Status Byte: 1 if sensor is working properly, 0 of not
 sensorStatusbyte
 0 Pre-heat temperature
 1 Post-heat temperature
 2 Pump temperature
 3 Pre-filter pressure
 4 Post-filter pressure
 5 Water fill pressure
 6 pump amps
 7 Water level sensor
 
 Discrete I/O status byte: shows on/off state if I/O
 ioStatusbyte
 0 Pump on/off relay
 1 Auto-Off-On switch is in Auto Position
 2 Auto-Off-On switch is in On Position
 3 Water fill LED
 4 Water fill pushbutton input
 5 Water fill valve relay
 6 Heater on/off relay output
 7 Water Level Sensor (real time)
 
 to set/clear a bits:
 statusbyte |= 1 << 5; will set bit 5 to 1
 statusbyte  &= ~(1 << 5); will clear bit 5,  invert the bit string with the bitwise NOT operator (~), then AND it.
 to read a bit: bitval = statusbyte & (1 << 5);  will but the value of bit 5 into bitval
 */

// Allocate array to hold bytes to send to other xbee.  Size is 2x the numer if integers being sent
uint8_t xbeePayload[NUM_DATA_PTS * 2];
// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request tx = Tx16Request(XBEE_MY_ADDR_RX, xbeePayload, sizeof(xbeePayload));
TxStatusResponse txStatus = TxStatusResponse();

uint32_t WaterFillTime;       // Timer to turn on water fill valve, adds 15 minutes every time pushbutton is pressed
byte    EmergencyShutdown;    // shuts down pump if there a problem.  0 = OK, 1 = low pressure, 2 = high amps, 3= high pump temperature
bool    presFluctResetFlag;   // Reset when pressure goes back to normal.  Used in conjunction with counter
bool    startupLowPressFlag;  // used to detect low pressure when pump start up in the morning.  SRG Wont need after water level sensor is installed

// Setup pushbutton for water fill.  Input goes low when pressed.
Button btnWaterFill = Button(WATER_FILL_PB, LOW);

bool waterFillOnTrigger;       // One shot trigger when water fill valve is turned on
byte sensorStatusbyte;        // Each bit determines if sensor is operating properly
byte ioStatusbyte;            // I/O state of digital I/O

// Function Prototype
bool getI2CData();
#ifdef PRINT_DEBUG
void PrintIO();
#endif

//============================================================================
//============================================================================
void setup ()
{
  
#ifdef PRINT_DEBUG
  Serial.begin(BAUD_RATE);
#endif
  
  pinMode(PUMP_OUTPUT,       OUTPUT);
  pinMode(WATER_FILL_PB_LED, OUTPUT);
  pinMode(WATER_OUTPUT,      OUTPUT);
  pinMode(PUMP_INPUT_AUTO,   INPUT_PULLUP);
  pinMode(PUMP_INPUT_MAN,    INPUT_PULLUP);
  pinMode(WATER_FILL_PB,     INPUT_PULLUP); // analog input used as digital input
  
  // Make sure outputs are off
  digitalWrite(PUMP_OUTPUT,       LOW);
  digitalWrite(WATER_FILL_PB_LED, LOW);
  digitalWrite(WATER_OUTPUT,      LOW);
  
  // start serial
  xbee.begin(BAUD_RATE);
  
  // Initialize I2C library - for panStamp communication and RTC
  Wire.begin();
  
  RTC.begin();   // Initialize Real Time Clock
  if (! RTC.isrunning())
  {
#ifdef PRINT_DEBUG
    Serial.println(F("RTC is NOT running!"));
#endif
  }
  
  // Start up the OneWire library
  waterTempSensors.begin();
  
  
  // following line sets the RTC to the date & time this sketch was compiled
  // RTC.adjust(DateTime(__DATE__, __TIME__));
  
  WaterFillTime = millis();
  waterFillOnTrigger = false;
  EmergencyShutdown = 0; // everything okay
  poolStatus = statusPumpOff;
  presFluctResetFlag = true; // set flag to true so no low pressue warning are given until pressure builds up
  startupLowPressFlag == false;  // Reset flag
  
#ifdef PRINT_DEBUG
  Serial.println(F("Finished setup()"));
#endif
  
  
} // setup()


void loop ()
{
  
//  static int ReadSensorCount;          // counts time sensor is read and is then used to take average
  static uint32_t SensorTimer;         // Timer for reading sensor every second
  static uint32_t TX_Timer;            // Timer to send data to house
  static uint32_t ReadTime;            // Timer for reading the Real Time Clock
  uint32_t xbeeWaitTime;               // Measures actual XBee response time, mS
//  static double TempPreHeater[2];      // Temperature before heater
//  static double TempPostHeater[2];     // Temperature after heater
//  static double TempPump[2];           // Temperature of pump housing
  static float temperature[3];         // Pre-heater temp (0), Post-Heter temp (1), pump temp (2)
  static float pressure[3];            // Pre-filter pressure (0), Post-filter pressure (1), water fill pressure (2)
  static double PumpAmps;              // Amps going to pump
//  static double Pressure1;             // Pressure before filter
//  static double Pressure2;             // Pressure after filter
//  static double Pressure3;             // Pressure at water fill valve
  static double poolTime;              // Time from Real Time Clock convted to decimal
  static double PressPreFilter;        // Pressure before filter, don't use Pressure1 because it's the cumulative ADC value until right before it's trasmitted to xbee
  static uint8_t waterAddedToday;      // minutes of water added today
  static uint32_t waterFillStart;      // Saves millis setting of when water fill timer starts
  static uint32_t debounceTimer;       // Used to prevent double entries of water fill pushbutton
  static uint32_t waterFillResetTime;  // Used to time how long the water fill pushbutton is held.  If over 2 seconds, then reset water fill
  static uint16_t presFluctCounter;    // Counts low pressure fluctuationsuint32_t LowPressTimer;       // Times how long the pressure is low, used to shutdown pump if there is a problem
  static uint32_t lowPressTimer;       // Times how long the pressure is low, used to shutdown pump if there is a problem
  static bool     lowPressTimerFlag;   // Flag for low pressure timer
  static int PumpAmpsAvg;              // Average pump amps calculation for emergency shutdown
  const byte NEW = 1;                  // Used in temperature smoothing
  const byte OLD = 0;                  // Used in temperature smoothing
  
  int16_t xbeeData[NUM_DATA_PTS];  // Array to hold integers that will be sent to other xbee
  
  
  // Get Time from RTC every 15 seconds
  // Combine hours and minutes into a decimal. i.e. 2:15:00 PM = 14.25
  if ((long)(millis() - ReadTime) >= 0 )
  {
    ReadTime = millis() + 15000;
    DateTime now = RTC.now();  // Gets the current time
    poolTime = (float) now.hour() + ((float) now.minute() / 60.0);
  }
  
  
  // Check low level sensor
  // If water fill valve is closed and there is water pressure and low level is detected, enable water fill valve by adding time the water fill timer
  if(analogRead(WATER_FILL_PRESSURE) > WATER_FILL_PRESS_THRESH &&
     digitalRead(WATER_OUTPUT) == LOW &&
     LowLevelSensor == 1 &&
     waterAddedToday <= MAX_WATER_FILL_MINUTES)
  {
    WaterFillTime = millis() + WATER_FILL_BP_MIN;
    
  }
  
  
  // Check for water fill pushbutton
  btnWaterFill.listen();
  if ( btnWaterFill.onPress() && waterAddedToday <= MAX_WATER_FILL_MINUTES && millis() > debounceTimer )
  {
    waterFillResetTime = millis();  // Used to time how long button is pressed, used for 2 second reset of water fill timer
    
    // Add time to water fill timer
    if(WaterFillTime <= millis() && analogRead(WATER_FILL_PRESSURE) > WATER_FILL_PRESS_THRESH && digitalRead(WATER_OUTPUT) == LOW)  // need to check for pressure here becuse if valve is already open, pressure will be low
    { // water fill is off, add 15 minutes to timer
      WaterFillTime = millis() + WATER_FILL_BP_MIN;  // First time button is pushed, add 15 minutes to timer
    }
    else if(digitalRead(WATER_OUTPUT) == HIGH)
    { // Water fill is already on, add another 15 minutes. Don't check water fill pressure because it will be low since valve is already open
      WaterFillTime += WATER_FILL_BP_MIN;          // Timer is already on (because output is on), add 15 more minutes
    }
    debounceTimer = millis() + 200;
  }
  
  // If button is held down for more then 2 seconds turn off water fill
  // The (waterFillResetTime > 0) statement prevents if statment from being true when arduino first boots up.
  // On bootup the .onRelease() returns true, even though button hasn't been pushed
  if(btnWaterFill.onRelease() && (millis() - waterFillResetTime) > 2000  && waterFillResetTime > 0)
  {
    WaterFillTime = millis();
  }
  
  // Turn on water fill valve
  if(((long)(millis() - WaterFillTime) < 0) && (waterAddedToday < MAX_WATER_FILL_MINUTES))
  {
    digitalWrite(WATER_OUTPUT, HIGH);
    if(poolStatus <= statusAddingWater)
    {
      poolStatus = statusAddingWater;
    }  // Set status to show water is filling
  }
  else
  { // Turn water fill off
    digitalWrite(WATER_OUTPUT, LOW);
    if(poolStatus == statusAddingWater)
    {
      poolStatus = 0;
    }  // Reset poolStatus when valve is turned off
  }
  
  // Turn on pushbutton LED when water fill valve is running
  digitalWrite(WATER_FILL_PB_LED, digitalRead(WATER_OUTPUT));  
  
  // Record time water fill valve is open
  // Check to see if water fill valve has just opened
  // Used to calculate time water fill valve was on - start time
  if(digitalRead(WATER_OUTPUT) == HIGH && waterFillOnTrigger == false)
  {
    // Valve just opened, set waterFillOnTrigger and set water fill start time
    waterFillOnTrigger = true;
    waterFillStart = millis();
  }
  
  // check to see if water fill valve has just closed
  // Used to calculate time water fill valve was on - end time
  if(digitalRead(WATER_OUTPUT) == LOW &&  waterFillOnTrigger == true)
  {
    // Valve just closed, reset waterFillOnTrigger
    waterFillOnTrigger = false;
    // Calculate time (minutes) that valve was on and to daily timer
    waterAddedToday += (int) ((millis() - waterFillStart) / 60000.0);
  }
  
  // If low pressure is detected, increase counter, only if water fill solenoid is off
  if(PressPreFilter < 13 &&
     presFluctResetFlag == false &&
     digitalRead(WATER_OUTPUT) == LOW &&
     digitalRead(PUMP_OUTPUT) == HIGH )
  {
    presFluctCounter++;
    presFluctResetFlag = true;
  }
  
  // Reset flag for low pressure counter once pre filter pressure increases
  if(PressPreFilter > 17 && presFluctResetFlag == true)
  {
    presFluctResetFlag = false;
  }
  
  // If low pressure counter > 20, add water to pool, 2x the pushbutton amount (30 min)
  // if water fill had been on for MAX_WATER_FILL_MINUTES minutes or less.
  // After that, don't add water and don't reset low pressure counter.
  if( presFluctCounter >= 20 &&
     digitalRead(WATER_OUTPUT) == LOW &&
     waterAddedToday < MAX_WATER_FILL_MINUTES &&
     analogRead(WATER_FILL_PRESSURE) > WATER_FILL_PRESS_THRESH )
  {
    WaterFillTime = millis() + WATER_FILL_BP_MIN + WATER_FILL_BP_MIN;  // Add water for 30 minutes
    presFluctCounter = 0;  // Reset low pressure counter; counter won't count up when water fill valve is on
    presFluctResetFlag == false;  // set to false so counter has chance to start again. If pressure never gets above 15, it won't reset
  }
  
  // Check for constant low pressure, not pressure fluctuations
  if( PressPreFilter < 11 &&
     digitalRead(PUMP_OUTPUT) == HIGH &&
     lowPressTimerFlag == false &&
     startupLowPressFlag == false )
  {
    lowPressTimer = millis() +  300000UL;  // start low pressure timer for 5 minutes (300k mS)
    lowPressTimerFlag = true;  // set flag so this lowPressTimer isn't updated again
  }

/*  
  // Check for low pressure when pump first starts up in morning. SRG don't need this after you setup low water level
  // detector in skimmer lid.
  // If time is between 1 and 5 minutes after morning start time and pressure is low, add 15 minutes of water
  if( digitalRead(PUMP_OUTPUT) == HIGH &&
     poolTime >= (PUMP_ON_TIME + 1.0/60.0) &&
     poolTime <= (PUMP_ON_TIME + 5.0/60.0)  &&
     PressPreFilter < 10  &&
     startupLowPressFlag == false )
  {
    startupLowPressFlag = true; // set flag so if statement only runs once
    WaterFillTime = millis() + WATER_FILL_BP_MIN + WATER_FILL_BP_MIN;  // Add water for 30
    lowPressTimer = WaterFillTime;    // don't let 5 minute shutdown happen
    Serial.println(F("Startup low pressure, turn water on"));
  }

  if ( poolTime >= PUMP_ON_TIME + 6.0/60.0)
  {
    startupLowPressFlag = false;
  }  // reset startuplowPress for next day.  SRG - don't need after low pressure sensor
  
*/

  // If pressure returns to normal or pump is off, reset low pressure timer flag
  if(PressPreFilter > 14 || digitalRead(PUMP_OUTPUT) == LOW)
  {
    lowPressTimerFlag = false;
  }
  
  // Shutdown pump if amps are too high
  // Need to reboot Arduino to restart
  if(PumpAmpsAvg > 20 && EmergencyShutdown == 0)
  {
    EmergencyShutdown = 3;
    poolStatus = statusEmergencyHiAmps;
  }
  
  // Shutdown pump if motor temp is too high
  // Need to reboot Arduino to restart
  if(temperature[PUMP_TEMP] > 180 && EmergencyShutdown == 0)
  {
    EmergencyShutdown = 2;
    poolStatus = statusEmergencyHiPumpTemp;
  }
  
  // Shutdown pump if low pressure counter goes to high
  // Counter will reset each time it reaches 20, and water is added.  But
  // after MAX_WATER_FILL_MINUTES minutes of water is added, it will not reset, but keep qoing up.
  // Or if water to the water fill line is off, then pressue will be low and water won't be added
  if(presFluctCounter > 25 && EmergencyShutdown == 0)
  {
    EmergencyShutdown = 1;
    poolStatus = statusEmergencyLoPresFluct;
  }
  
  // If pressure has been low for 5 minutes straight, shut down pump
  if(long(millis() - lowPressTimer) > 0 && lowPressTimerFlag == true && EmergencyShutdown == 0)
  {
    EmergencyShutdown = 1;
    poolStatus = statusEmergencyLoPresCont;
  }
  
  // Turn pump on if:
  //   It's daytome and pump switch is in auto
  //   Or Pump switch is in manual on
  if(((poolTime >= PUMP_ON_TIME) && (poolTime <= PUMP_OFF_TIME) && (digitalRead(PUMP_INPUT_AUTO) == PUMP_SWITCH_ON) && (EmergencyShutdown == 0)) || (digitalRead(PUMP_INPUT_MAN) == PUMP_SWITCH_ON))
  {
    // Turn Pump On
    digitalWrite(PUMP_OUTPUT, HIGH);
    if(poolStatus <= statusPumpSwitchOff)
    {
      poolStatus = statusPumpOn;
    }   // set pool status to Pump On, don't override higher status codes
  }
  else
  { // Turn Pump off
    digitalWrite(PUMP_OUTPUT, LOW);
    if(poolStatus <= statusPumpSwitchOff)
    {
      poolStatus = statusPumpOff;
    }  // set pool status to Pump Off, don't override higher status codes
  }
  
  // See if pump on/off switch is in off position, if so, set poolStatus and reset EmergencyShutdown
  if(digitalRead(PUMP_INPUT_AUTO) != PUMP_SWITCH_ON && digitalRead(PUMP_INPUT_MAN) != PUMP_SWITCH_ON)
  {
    poolStatus = statusPumpSwitchOff;
    EmergencyShutdown = 0;
  }
  
  
  // Read Sensors
  // Temperature is smoothed with low pass filter
  // Other sensores are summed up, then averaged over the Xbee upload interval and converted when ready to uplod
  if ((long)(millis() - SensorTimer) >= 0 )
  { // Read Sensors
    SensorTimer = millis() + SENSOR_READ_INTERVAL;
/*
    ReadSensorCount++;  // Increment sensor read counter - used in averaging the values
    
    waterTempSensors.requestTemperatures();   // Send the command to get temperatures
    TempPreHeater[NEW]  = waterTempSensors.getTempF(&tempSensors[0][0]);
    TempPostHeater[NEW] = waterTempSensors.getTempF(&tempSensors[1][0]);
    TempPump[NEW] =       waterTempSensors.getTempF(&tempSensors[2][0]);
    
    // Low pass filter to smooth temperature data, http://bit.ly/Ov9r28
    float Smoothing = 0.3;  // smaller gives more smoothing, range 0 to 1.  1 is no smoothing
    TempPreHeater[NEW]   = (Smoothing * TempPreHeater[NEW]) +   ((1.0 - Smoothing) * TempPreHeater[OLD]);
    TempPostHeater[NEW]  = (Smoothing * TempPostHeater[NEW]) +  ((1.0 - Smoothing) * TempPostHeater[OLD]);
    TempPump[NEW]        = (Smoothing * TempPump[NEW]) +        ((1.0 - Smoothing) * TempPump[OLD]);
    TempPreHeater[OLD]   = TempPreHeater[NEW];
    TempPostHeater[OLD]  = TempPostHeater[NEW];
    TempPump[OLD]        = TempPump[NEW];
    
    // Accumulate values for amps and Pressures
    PumpAmps    += (double) analogRead(PUMP_AMPS_PIN);
    Pressure1   += (double) analogRead(PRESSURE1_PIN);
    Pressure2   += (double) analogRead(PRESSURE2_PIN);
    Pressure3   += (double) analogRead(WATER_FILL_PRESSURE);
*/
    // Read temperature sensors and use low pass filter to smooth
    float Smoothing = 0.3;  // smaller gives more smoothing, range 0 to 1.  1 is no smoothing
    waterTempSensors.requestTemperatures();   // Send the command to get temperatures
    for (byte i = 0; i < 3; i++)
    { temperature[i] = (Smoothing * waterTempSensors.getTempF(&tempSensors[i][0])) + ((1.0 - Smoothing) * temperature[i]); }

    // Read pressure sensors and use low pass filter to smooth
    float newPressure;
    newPressure = 0.0343 * (float) analogRead(PRESSURE1_PIN) - 5.9077;  // srg: 12.5 psi = 500 ADC, 0 PSI = 193 ADC, calc seems to be about 1.5 PSI Low
    if (newPressure < 0.7)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    pressure[PRE_FILTER_PRESSURE] = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[PRE_FILTER_PRESSURE]);
    
    newPressure = 0.0359 * (float) analogRead(PRESSURE2_PIN) - 6.2548;
    if (newPressure < 0.7)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    pressure[POST_FILTER_PRESSURE] = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[POST_FILTER_PRESSURE]);


    // Water fill pressure could use two different sensors, 0-30 PSI or 0-100 PSI
    if (analogRead(WATER_FILL_PRESSURE) > 900 )
    { // using 0-30 PSI sensor
      newPressure = 0.0359 * (float) analogRead(WATER_FILL_PRESSURE) - 6.2548;
    }
    else
    { // using 0-100 PSI sensor
      newPressure = 0.12225 * (float) analogRead(WATER_FILL_PRESSURE) - 25.061;
    }
    
    if (newPressure < 0.7)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    pressure[WATER_FILL_PRESSURE]  = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[WATER_FILL_PRESSURE]);
    
    // Read pump amps and use low pass filter
    PumpAmps = (Smoothing * (float) analogRead(PUMP_AMPS_PIN)) + ((1.0 - Smoothing) *  PumpAmps);
  }
  

  // Calculate averages and send data to Arduino in the house via XBee
  if((long)(millis() - TX_Timer) >= 0 )
  {
    TX_Timer = millis() + TX_INTERVAL;
    
    // Request I2C data from panStamp
    getI2CData();
/*
    // Calculate averages and perform ADC conversions
    PumpAmps       = PumpAmps / ReadSensorCount;
    PumpAmps       = PumpAmps * 0.0185;
    if (PumpAmps  < 0.5) PumpAmps =  0.0;
    PumpAmpsAvg = PumpAmps;  // Aveage Pump amps used for emergency shutdown
    
    Pressure1      = Pressure1 / (double) ReadSensorCount;
    Pressure1      = 0.0343 * Pressure1 - 5.9077;  // srg: 12.5 psi = 500 ADC, 0 PSI = 193 ADC, calc seems to be about 1.5 PSI Low
    PressPreFilter = Pressure1; // Used to detect low pressure for water fill detection
    Pressure2      = Pressure2 / (double) ReadSensorCount;
    Pressure2      = 0.0359 * Pressure2 - 6.2548;
    Pressure3      = Pressure3 / (double) ReadSensorCount;
    Pressure3      = 0.0359 * Pressure3 - 6.2548;
    
    
    if (Pressure1 < 0.7) Pressure1 = 0.0;
    if (Pressure2 < 0.7) Pressure2 = 0.0;
    if (Pressure3 < 0.7) Pressure3 = 0.0;
*/
    
    // Set bits for Sensor working properly or not
    
    // Validate temps recieved
//   if (TempPreHeater[NEW] < 50 || TempPreHeater[NEW]  > 180)
   if (temperature[PRE_HEAT_TEMP] < 50 || temperature[PRE_HEAT_TEMP] > 180)
    {
      sensorStatusbyte  &= ~(1 << 0);
    }  // Invalid temp range, something wrong with sensor
    else
    {
      sensorStatusbyte |= 1 << 0;
    }  // valid temp range
   
//   if (TempPostHeater[NEW] < 50 || TempPostHeater[NEW]  > 180)
   if (temperature[POST_HEAT_TEMP] < 50 || temperature[POST_HEAT_TEMP] > 180)
    {
      sensorStatusbyte  &= ~(1 << 1);
    }  // Invalid temp range, something wrong with sensor
    else
    {
      sensorStatusbyte |= 1 << 1;
    }  // valid temp range
   
//   if (TempPump[NEW] < 40 || TempPump[NEW]  > 300)
   if (temperature[PUMP_TEMP] < 40 || temperature[PUMP_TEMP] > 300)
    {
      sensorStatusbyte  &= ~(1 << 2);
    }  // Invalid temp range, something wrong with sensor
    else
    {
      sensorStatusbyte |= 1 << 2;
    }  // valid temp range
    
    // Pre filter pressure
    // If pump is on and pressure2 is okay (>5), but higher then pressure1, then there is a problem with presssure1 (pre-filter)
//   if (digitalRead(PUMP_OUTPUT) == HIGH && Pressure2 > 5 && Pressure2 > Pressure1)
   if (digitalRead(PUMP_OUTPUT) == HIGH && pressure[POST_FILTER_PRESSURE] > 5 && pressure[POST_FILTER_PRESSURE] > pressure[PRE_FILTER_PRESSURE])
    {
      sensorStatusbyte  &= ~(1 << 3);
    }  // Invalid pre-filter pressure
    else
    {
      sensorStatusbyte |= 1 << 3;
    }  // valid pre-filter pressure range
    
    // Post filter pressure
    // If pump is on and pre-filter pressure is okay, but post filter pressure is low, then there is problem with sensor
    // Note - when discharging water to waste, this will cause pressure2 to show an problem
//   if (digitalRead(PUMP_OUTPUT) == HIGH && Pressure1 > 10 && Pressure2 < 4)
   if (digitalRead(PUMP_OUTPUT) == HIGH && pressure[PRE_FILTER_PRESSURE] > 10 && pressure[POST_FILTER_PRESSURE] < 4)
    {
      sensorStatusbyte  &= ~(1 << 4);
    }  // Invalid pressure
    else
    {
      sensorStatusbyte |= 1 << 4;
    } // valid pressure range
    
    // water fill pressure
    // Sensor outputs 1-5 volts, so min ADC should be about 200, so if it's less then 100, there is definitely a problem
    if(digitalRead(WATER_FILL_PRESSURE) < 100)
    {
      sensorStatusbyte |= 1 << 5;
    }   // sensor ok (1)
    else
    {
      sensorStatusbyte  &= ~(1 << 5);
    } // Sensor failed (0)
    
    // Pump Amps
    // if pump is on but amps are low, there is a problem with sensor (or pump)
    if (digitalRead(PUMP_OUTPUT) == HIGH && PumpAmps < 4 )
    {
      sensorStatusbyte  &= ~(1 << 6);
    }  // Invalid pump amps
    else
    {
      sensorStatusbyte |= 1 << 6;
    }     // pump amps okay
    
    // if pump is off but amps are high, there is a problem with sensor (or pump)
    if (digitalRead(PUMP_OUTPUT) == LOW && PumpAmps > 3)
    {
      sensorStatusbyte  &= ~(1 << 6);
    }  // Invalid pump amps
    else
    {
      sensorStatusbyte |= 1 << 6;
    } // pump amps okay
    
    // Water level sensor is set in getI2CData()
        
    
    // set bits in ioStatusbyte 
    // shows input value for each I/O
    if(digitalRead(PUMP_OUTPUT) == HIGH)
    {
      ioStatusbyte |= 1 << 0;
    }    // pump is on
    else
    {
      ioStatusbyte &= ~(1 << 0);
    }  // pump is off
    
    if(digitalRead(PUMP_INPUT_AUTO) == LOW)
    {
      ioStatusbyte |= 1 << 1;
    }    // 3-posistion switch is in Auto mode
    else
    {
      ioStatusbyte &= ~(1 << 1);
    }  // 3-posistion switch not in Auto mode
    
    if(digitalRead(PUMP_INPUT_MAN) == LOW)
    {
      ioStatusbyte |= 1 << 2;
    }    // 3-posistion switch is in manual mode
    else
    {
      ioStatusbyte &= ~(1 << 2);
    }  // 3-posistion switch not in manual mode
    
    if(digitalRead(WATER_FILL_PB_LED) == HIGH)
    {
      ioStatusbyte |= 1 << 3;
    }    // water fill LED is on
    else
    {
      ioStatusbyte &= ~(1 << 3);
    }  // water fill LED is off
    
    if(digitalRead(WATER_FILL_PB) == LOW)
    {
      ioStatusbyte |= 1 << 4;
    }    // water fill pushbutton is being pressed
    else
    {
      ioStatusbyte &= ~(1 << 4);
    }  // water fill pushbutton is not pressed
    
    if(digitalRead(WATER_OUTPUT) == HIGH)
    {
      ioStatusbyte |= 1 << 5;
    }    // water fill relay is on
    else
    {
      ioStatusbyte &= ~(1 << 5);
    }  // water fill relay is off
    
    ioStatusbyte &= ~(1 << 6);     // set heater relay status to off for now because it doesn't exist yet
    
    // Water level bit 7 is set in getI2CData()
    
    
    // Put data in xbeeData array, multiple by 10 so you can get 1 decimal point, divide by 10 on the Rx side
/*  xbeeData[0] = (int) (TempPreHeater[NEW]  * 10.0);
    xbeeData[1] = (int) (TempPostHeater[NEW] * 10.0);
    xbeeData[2] = (int) (TempPump[NEW]       * 10.0);
    xbeeData[3] = (int) (PumpAmps            * 10.0);
    xbeeData[4] = (int) (Pressure1           * 10.0);
    xbeeData[5] = (int) (Pressure2           * 10.0);
    xbeeData[6] = (int) (Pressure3           * 10.0);
*/
    xbeeData[0] = (int) (temperature[PRE_HEAT_TEMP]     * 10.0);
    xbeeData[1] = (int) (temperature[POST_HEAT_TEMP]    * 10.0);
    xbeeData[2] = (int) (temperature[PUMP_TEMP]         * 10.0);
    xbeeData[3] = (int) (PumpAmps                       * 10.0);
    xbeeData[4] = (int) (pressure[PRE_FILTER_PRESSURE]  * 10.0);
    xbeeData[5] = (int) (pressure[POST_FILTER_PRESSURE] * 10.0);
    xbeeData[6] = (int) (pressure[WATER_FILL_PRESSURE]  * 10.0);
    xbeeData[7] = presFluctCounter      * 10;  // counts low pressure fluctuations, means water level is low or leaves in filter
    xbeeData[8] = (int) (poolStatus     * 10);
    xbeeData[9] = (int) (waterAddedToday * 10);  // Number of minutes fill water valve was open
    if (WaterFillTime > millis())  // send water fill countdown to xBee.  If timer is not running, send zero
    { xbeeData[10] = ((WaterFillTime - millis()) / 6000.0); }
    else
    { xbeeData[10] = 0; }
    xbeeData[11] = (int) (poolTime * 10.0);
    xbeeData[12] = (int) (LevelSensorBatt * 10.0);  // battery volts (in mV) for water level sensor
    xbeeData[13] = (int) LowLevelSensor   * 10.0;
    xbeeData[14] = (int) sensorStatusbyte * 10.0;
    xbeeData[15] = (int) ioStatusbyte     * 10.0;
    
    // Send data to house
    // break down integers into two bytes and place in payload
    for(int i=0; i < NUM_DATA_PTS; i++)
    {
      xbeePayload[i*2]     = xbeeData[i] >> 8 & 0xff; // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
      xbeePayload[(i*2)+1] = xbeeData[i] & 0xff;      // low byte, just mask off the upper 8 bits
    }
    xbee.send(tx);
    
    // after sending a tx request, we expect a status response
    // wait up to 1 seconds for the status response
    xbeeWaitTime = millis(); // use to measure actual XBee wait time
    if (xbee.readPacket(ACK_RESPONSE_WAIT_TIME))
    {
      // got a response!
      if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)
      {
        xbee.getResponse().getZBTxStatusResponse(txStatus);
        
        // get the delivery status, 0 = OK, 1 = Error, 2 = Invalid Command, 3 = Invalid Parameter
        if (txStatus.getStatus() == SUCCESS)
        {
          // success.  time to celebrate
        }
        else
        {
          // the remote XBee did not receive our packet. is it powered on?
        }
      }
    }
    else if (xbee.getResponse().isError())
    {
      #ifdef PRINT_DEBUG
        Serial.print(F("Error reading packet.  Error code: "));
        Serial.println(xbee.getResponse().getErrorCode());
      #endif
    }
    else
    {
      // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
      #ifdef PRINT_DEBUG
        Serial.println(F("XBee did not provide a timely Tx Status Response"));
      #endif
      
    }  // Finished waiting for XBee packet
    
    // Clear sensor data
/*    PumpAmps        = 0;
    Pressure1       = 0;
    Pressure2       = 0;
    Pressure3       = 0;
    ReadSensorCount = 0;
*/    
  }  // TX_Timer
  
  // Reset waterAddedToday and low pressure counter every night at 11PM
  if (poolTime > 23.0)
  {
    waterAddedToday = 0;
    presFluctCounter = 0;
  }
  
} // loop()



// =============================================================
// I2C Request data from panStamp slave
bool getI2CData()
{
  bool gotI2CPacket = false;
  byte i=0;  // counts packets received from panStamp
  byte i2CData[I2C_PACKET_SIZE];  // don't use char data type
  
  Wire.requestFrom(addrSlaveI2C, I2C_PACKET_SIZE);    // request data from I2C slave
  
  while(Wire.available())    // Wire.available() will return the number of bytes available to read
  {
    i2CData[i++] = Wire.read(); // receive a byte of data
    gotI2CPacket = true;  // Flag to indicate sketch received I2C packet
  }
  
  // If we got an I2C packet, then extract the data from the I2C packet
  if(gotI2CPacket)
  {
    gotI2CPacket = false;  // Reset flag
    
    // if panstamp TX is online, extract data, if not return offline codes
    if (i2CData[1] == 0 )
    { // water level detector TX is online - low water for 2 minutes
      sensorStatusbyte |= 1 << 7;   // set water level sensor bit  to indicate sensor is online
      LowLevelSensor = i2CData[2];  // Set global variable, water low for 2 minutes
      
      // set ioStatusbyte bit for water level sensor (real time value)
      if(i2CData[3] == true )
      { ioStatusbyte |= 1 << 7; }    // water level is low, set bit
      else
      { ioStatusbyte &= ~(1 << 7); } // water level is okay, clear bit
      
      // Level Lid - i2CData[4] - not used by anything in this sketch
      
      // Battery voltage
      LevelSensorBatt = i2CData[5] << 8;
      LevelSensorBatt |= i2CData[6];
      
    }
    else
    { // water level detector TX is offline
      sensorStatusbyte  &= ~(1 << 7);  // clear water level sensor bit to indicate sensor is offline
      LowLevelSensor = 2;  // 0 - level ok, 1 - level low, 2 - offline
      LevelSensorBatt = 0;
    }
    return true;
  }  // end if(gotI2CPacket)
  else
  { 
    return false;  // No Packet received
  } 
  
} // getI2CData()



