/* 
Main hardware: Arduino Leonardo, Xbee, RTC and panStamp
This sketch run on the Leonardo
 
GitHub Repo: http://git.io/xzlBSQ
Xively: https://personal.xively.com/feeds/65673/

To do: 
Calibrate pressure sensors
Display errors on OLED
Upgrade to Mega
Add bluetooth to program wirelessly
Put g_ prefix on global variables
Try to detect when level sensor is stuck
 
Change log
07/01/14 v1.50 - Added delay in pump off to stop false positives for pump amp sensor status. Changed xbee.begin(9600); 
                 and restored old xbee library.  Having problems with new library from Feb 2014
07/02/14 v1.51 - Added condition to check for Leonardo and to use Serial1 instead of Serial for Xbee so it would work
                 with v0.5 of the xbee library
                 Changed logic to prevent false pump amps sensor alert
07/02/14 v1.52 - fixed bug with presFluctResetFlag, it wasn't getting reset.  Still getting false positive for 
                 amp sensor when motor is shut off, so I removed code that chekcs for amps with the motor off
07/07/14 v1.53 - Removed commented code that checked for low pressure, but it's not needed since the low level
                sensor was installed
07/24/14 v1.54 - Changed water fill max from 120 to 60, formatting,
08/03/14 v1.55 - changed how minutes per day of water fill is tracked. Will show each minute instead of 15-minute steps.
                 Added enum waterLevel_t
08/17/14 v1.56 - Added checksum to I2C. Changed live level sensor so level ok = 0. Replaced water countdown with flat 
                 lid in Xbee packet byte 10
08/18/14 v1.57 - Changed PANSTAMP_ONLINE from 0 to 1.  Getting a lot of checksum errors.  When there's an error every
                 element but checksum is 0, so I didn't want 0 to be a valid value for PANSTAMP_ONLINE
                 This also seemed to fix problem where the sensorstatus byte was sometimes reporting level sensor as 
                 being okay (bit 7), when I had it disconnected
09/07/14 v1.58 - Increased max water fill time from 60 to 90 minutes. Added statistic.h library to calc max and stdev 
                 of pre-filter pressure in order to better detect oscillations
                 Added enum emergencyStatus_t & poolStatus_t
09/17/14 v1.59 - Moved I/O pins to be compatible with new PCB
09/21/14 v1.60 - Added code to support OLED display.  Getting low on memory, should probably upgrade to Arduino Mega
05/21/15 v1.61 - Renamed currenlyDisplaying to currentlyDisplaying
05/28/15 v1.62 - Added reset low pressure alarm at end of the day, converted pressure and temperature array index from #define to enum
06/15/15 v1.63 - Added code to detect failed pre-filter pressure sensor
09/01/15 v1.64 - Put ifdef ENABLE_OLED around OLED code to save memore. Memory too low on Leonardo.
                 Have problem where 5 min low pressure alarm would sometimes come on after I shut off pump when changing backpack, I hope the two changes below fix this 
                 1) Changed isLowPressTimerRunning trigger so it resets if it's 2 PSI above threshold, was 3 PSI.
                 2) Added preFilterPressureStats.clear() when pump is turned off
*/

#define VERSION "v1.64"

#define PRINT_DEBUG   // Comment out if not debugging
// #defing ENABLE_OLED   // Not enought memory on Leonardo, you can uncomment if you use a bigger Arduino

#include "Pool_Controller_Outside_Library.h"
#include <Wire.h>              // http://www.arduino.cc/en/Reference/Wire
#include <RTClib.h>            // http://github.com/adafruit/RTClib
#include <XBee.h>              // http://code.google.com/p/xbee-arduino/     Modified per http://arduino.cc/forum/index.php/topic,111354.0.html
#include <Button.h>            // For pushbutton http://github.com/carlynorama/Arduino-Library-Button
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include "Statistic.h"         // http://playground.arduino.cc/Main/Statistics
#include <SPI.h>
#ifdef ENABLE_OLED
  #include <Adafruit_GFX.h>      // http://github.com/adafruit/Adafruit-GFX-Library
  #include <Adafruit_SSD1306.h>  // http://github.com/adafruit/Adafruit_SSD1306
#endif

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// === Analog I/O Pins ===
#define PRESSURE1_PIN            0   // Pressure before filter
#define PRESSURE2_PIN            1   // Pressure after filter
#define WATER_FILL_PRESSURE_PIN  2   // Pressure at water fill line.  Transducer can be 0-30 or 0-100 PSI transducer
#define PUMP_AMPS_PIN            3   // Pump amps input 20 Amp CT
#define WATER_FILL_PB           A4   // Push-button for water fill, use as digital input - fills water 15 minutes each time it's pressed
// A5 Unused

// === Digital I/0 Pins ===
// Pins 0 & 1 are used to communicate with xBee
// Pins 2 and 3 are used by Lonardo for I2C communication
// D4 unused
#define ONE_WIRE_BUS         5   // 1-Wire temperature sensor bus
#define OLED_RESET           6   // OLED display reset
#define HEAT_OUTPUT          7   // Turn on heater - future use
// D8 Reserved for remote LED display (if reset pin is needed)
#define PUMP_OUTPUT          9   // Turns pump on/off
#define WATER_FILL_PB_LED   10   // LED on Water Fill Pushbutton
#define WATER_OUTPUT        11   // Controls water fill solenoid valve
#define PUMP_INPUT_AUTO     12   // From On-Off_Auto switch, Auto Mode
#define PUMP_INPUT_MAN      13   // From On-Off_Auto switch, On Mode


enum index_temperature_t {
  PRE_HEAT_TEMP,       // Pre-heater temperature array
  POST_HEAT_TEMP,      // Post-heater temperature array
  PUMP_TEMP,           // Pump temperature array
  ARRAY_TEMP_NUM_POSIT // Number of positions in temperature array
};
float temperature[ARRAY_TEMP_NUM_POSIT];

enum index_pressure_t
{
  PRE_FILTER_PRESSURE,   // Pre-filter water pressure array
  POST_FILTER_PRESSURE,  // Post-filter water pressure array
  WATER_FILL_PRESSURE,   // Water fill pressure array
  ARRAY_PRES_NUM_POSIT   // Number of positions in pressure array
};
float pressure[ARRAY_PRES_NUM_POSIT];

#define PUMP_SWITCH_ON   LOW   // Pump switch is ON when input is LOW, otherwise it's high from the pull-up resistor

const uint32_t SENSOR_READ_INTERVAL =     250;  // Read sensor every 250mS
const uint32_t TX_INTERVAL =             2000;  // Send data to house every 2 seconds
const uint32_t ACK_RESPONSE_WAIT_TIME =   250;  // Time (mS) program waits for an ACK response from other XBee. Normal response time is about 15mS
const float    PUMP_ON_TIME =             7.5;  // 7:30 AM, Time to turn pump on each day
const float    PUMP_OFF_TIME =           19.0;  // 7:00 PM, Time to turn pump off each day
#define        XBEE_MY_ADDR_RX          0x250   // The MY address of the Rx XBee (don't use const byte). Don't care what the MY address is of Tx as long as it's not the same as Rx
const uint32_t WATER_FILL_15_MIN =     908000;  // 15 minutes added to water fill timer
const byte     WATER_FILL_PRESS_THRESH =   20;  // If water fill pressure is > then thershold, you can assume garden hose is connected to water fill
const byte     PRESSURE_TRANSDUCER_TYPE = 100;  // Can use either 100 PSI sensor or 30 PSI sensor
                                                // With 0-30PSI transducer, analog input = 1023 at with valve off, 290 with valve open, 190 with hose diconnected
const byte MAX_WATER_FILL_MINUTES =        90;  // Maximum number of minutes water can be added each day.  Reset at 11 PM. Refactored some of the comples if() statements
const byte PANSTAMP_ONLINE =                1;  // panStamp status, 0 = online, 255 = offline
const uint32_t SAMPLE_TIME =            10000;  // Sample time for pre-filter pressure - used in statisic library

// Status code to send to inside Arduino
// Indicated current state of poolc controller
enum poolStatus_t
{
  STATUS_PUMP_OFF,
  STATUS_PUMP_ON,
  STATUS_PUMP_SWITCH_OFF,
  STATUS_ADDING_WATER,
  STATUS_EMERGENCY_LOW_PRESS_FLUCT,  // low pressure because of fluctuations
  STATUS_EMERGENCY_LOW_PRESS_COUNT,  // continuous low pressure for 5 minutes
  STATUS_EMERGENCY_HI_AMPS,
  STATUS_EMERGENCY_HI_PUMP_TEMP
};
poolStatus_t poolStatus;

enum waterLevel_t { WATER_LEVEL_OK, LOW_WATER, LEVEL_SENSOR_OFFLINE };
waterLevel_t lowWaterLevel = WATER_LEVEL_OK;   // Water level ok = 0, water level low = 1, sensor offline = 2
uint16_t levelSensorMilliVolts = 0; // Battery voltage (mV) for level sensor
bool isLidLevel = false; // Level sensor lid, flat = true

// Initialize Real Time Clock
RTC_DS1307 RTC;

#ifdef ENABLE_OLED
  Adafruit_SSD1306 display(OLED_RESET);
#endif

// Statistics for prefilter pressure
Statistic preFilterPressureStats;

const byte ADDR_SLAVE_I2C =  21;  // I2C Slave address of RX panStamp
const byte I2C_PACKET_SIZE = 15;  // I2C Packet size


// Initialize OneWire temp sensors
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature waterTempSensors(&oneWire);
static uint8_t tempSensors[3][8] =
{
  { 0x10, 0xD0, 0x8E, 0x6A, 0x02, 0x08, 0x00, 0xE4 },  // Pre-heater temperature sensor
  { 0x10, 0xCB, 0x95, 0x6A, 0x02, 0x08, 0x00, 0x4B },  // Post-heater temperature sensor
  { 0x10, 0x07, 0x91, 0x6A, 0x02, 0x08, 0x00, 0xB6 }   // Pump housing temperature sensor
};


// Intialize Xbee object
XBee xbee = XBee();
#define NUM_DATA_PTS 16  // Number of integers (data points) to upload. Can't exceed 100 bytes or 50 integers unless you change MAX_FRAME_DATA_SIZE in XBee.h

// Allocate array to hold bytes to send to other xbee.  Size is 2x the number if integers being sent
uint8_t xbeePayload[NUM_DATA_PTS * 2];
// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request tx = Tx16Request(XBEE_MY_ADDR_RX, xbeePayload, sizeof(xbeePayload));
TxStatusResponse txStatus = TxStatusResponse();

uint32_t WaterFillTimer;      // Timer to turn on water fill valve, adds 15 minutes every time pushbutton is pressed
enum emergencyStatus_t {EVERYTHING_OK, LOW_PRESSURE, HIGH_AMPS, HIGH_PUMP_TEMP};
emergencyStatus_t  EmergencyShutdown;    // shuts down pump if there a problem.  0 = OK, 1 = low pressure, 2 = high amps, 3 = high pump temperature
bool    presFluctResetFlag;   // Reset when pressure goes back to normal.  Used in conjunction with counter


// Setup pushbutton for water fill.  Input goes low when pressed.
Button btnWaterFill = Button(WATER_FILL_PB, LOW);

byte sensorStatusByte;           // Each bit determines if sensor is operating properly
byte ioStatusByte;               // I/O state of digital I/O
int16_t xbeeData[NUM_DATA_PTS];  // Array to hold integers that will be sent to other xbee
float PumpAmps = 0;              // Amps going to pump
float poolTime;                  // Time from Real Time Clock convted to decimal
uint8_t waterAddedToday = 0;     // minutes of water added today
uint16_t presFluctCounter = 0;   // Counts low pressure fluctuations


// Function Prototype
bool getI2CData();
void setIoStatusByte();
bool sendXbeeData();
void updateDisplay();
void printDebugFunction(bool lowPressTmr, float lowPresThreshold);
bool isNewMinute();
bool isNewDay();


//============================================================================
//============================================================================
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);    // for Leonardo, need to use Serial1, not Serial
  xbee.setSerial(Serial1);
delay(4000);//srgg

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
  
  
  // Initialize I2C library - for panStamp communication and RTC
  Wire.begin();

  // Initialize Real Time Clock
  RTC.begin();
  #ifdef PRINT_DEBUG
    if (! RTC.isrunning())
     { Serial.println(F("RTC is NOT running!")); }
  #endif
  
  
  // Start up the OneWire library
  waterTempSensors.begin();
  
  
  // following line sets the RTC to the date & time this sketch was compiled
  // To initially set the clock, uncomment the line below.  Compile and upload.  Then comment the
  // line out and upload again.
  // RTC.adjust(DateTime(__DATE__, __TIME__));
  
  WaterFillTimer = millis();
  EmergencyShutdown = EVERYTHING_OK;
  poolStatus = STATUS_PUMP_OFF;
  presFluctResetFlag = true; // set flag to true so no low pressue warning are given until pressure builds up
//  startupLowPressFlag == false;  // Reset flag
  
  preFilterPressureStats.clear(); // Clear statistic stats

#ifdef ENABLE_OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  display.clearDisplay();  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("Pool Controller"));
  display.println(F("Version "));
  display.println(VERSION);
  display.display();
#endif

#ifdef PRINT_DEBUG
  Serial.print(F("Finished setup() "));
  Serial.println(VERSION);
#endif
  
} // setup()


//============================================================================
//============================================================================
void loop()
{
  static uint32_t SensorTimer;          // Timer for reading sensor every second
  static uint32_t TX_Timer;             // Timer to send data to house
  static uint32_t ReadTime;             // Timer for reading the Real Time Clock
  static uint32_t debounceTimer;        // Used to prevent double entries of water fill pushbutton
  static uint32_t waterFillResetTime;   // Used to time how long the water fill pushbutton is held.  If over 2 seconds, then reset water fill
  static uint32_t lowPressTimer;        // Times how long the pressure is low, used to shutdown pump if there is a problem
  static uint32_t statsSampleTimer = 0; // Timer for pre-filter pressure sampling
#ifdef ENABLE_OLED
  static uint32_t displayTimer = 0;     // displays each screen for a few seconds
#endif
  static bool     isLowPressTimerRunning;   // Flag for low pressure timer, true if low pressure timer has started
  static float    lowPreFilterPressureThreshold = 15; // Pressure setting used to count pressure fluctuations when pump is starved for water.  Threshold is adjusted by max operating pressure.
  
  
  // Get Time from RTC every 15 seconds
  // Combine hours and minutes into a decimal. i.e. 2:15:00 PM = 14.25
  if ((long)(millis() - ReadTime) >= 0 )
  {
    ReadTime = millis() + 15000UL;
    DateTime now = RTC.now();  // Gets the current time
    poolTime = (float) now.hour() + ((float) now.minute() / 60.0);
  }
  

  // Check pool water level. Remote water level sensor wirelessly sends status 
  // If water fill valve is closed AND there is water pressure AND low level is detected AND haven't exceeded daily water fill time, 
  // THEN enable water fill valve by adding time to the water fill timer
  bool isWaterFillPressureOkay = pressure[WATER_FILL_PRESSURE] > WATER_FILL_PRESS_THRESH;
  bool isWaterFillValveClosed = digitalRead(WATER_OUTPUT) == LOW;
  bool isWaterLevelLow = lowWaterLevel == LOW_WATER;
  bool isOkToAddWater = waterAddedToday <= MAX_WATER_FILL_MINUTES;
  if ( isWaterFillPressureOkay && isWaterFillValveClosed && isWaterLevelLow && isOkToAddWater )
  { WaterFillTimer = millis() + WATER_FILL_15_MIN; } // this will turn on water fill
    
  // Check for water fill pushbutton
  // Quick push will turn on water fill valve.  Holding pushbutton in will cancel water fill operation
  btnWaterFill.listen();
  if ( btnWaterFill.onPress() && isOkToAddWater && millis() > debounceTimer )
  {
    waterFillResetTime = millis();  // Used to time how long button is pressed, if pressed for 2 second then reset of water fill timer
    
    // Add time to water fill timer
    if( WaterFillTimer <= millis() && pressure[WATER_FILL_PRESSURE] > WATER_FILL_PRESS_THRESH && digitalRead(WATER_OUTPUT) == LOW )  // need to check for pressure here becuse if valve is already open, pressure will be low
    {  WaterFillTimer = millis() + WATER_FILL_15_MIN; } // Water fill is off, add 15 minutes to timer
    else if( digitalRead(WATER_OUTPUT) == HIGH )
    { WaterFillTimer += WATER_FILL_15_MIN; } // Water fill is already on, add another 15 minutes. Don't check water fill pressure because it will be low since valve is already open
    debounceTimer = millis() + 200;
  }

  // If button is held down for more then 2 seconds turn off water fill
  // The (waterFillResetTime > 0) statement prevents if statment from being true when arduino first boots up.
  // On bootup the .onRelease() returns true, even though button hasn't been pushed
  if( btnWaterFill.onRelease() && (millis() - waterFillResetTime) > 2000UL  && waterFillResetTime > 0 )
  { WaterFillTimer = millis(); }

  // Turn on water fill valve
  if( ((long)(millis() - WaterFillTimer) < 0) && isOkToAddWater )
  {
    digitalWrite(WATER_OUTPUT, HIGH);
    if( poolStatus <= STATUS_ADDING_WATER )
    { poolStatus = STATUS_ADDING_WATER; } // Set status to show water is filling 
  }
  else
  { // Turn water fill off
    digitalWrite(WATER_OUTPUT, LOW);
    if( poolStatus == STATUS_ADDING_WATER )
    { poolStatus = STATUS_PUMP_OFF; } // Reset poolStatus when valve is turned off
  }
 
  // Turn on pushbutton LED when water fill valve is running
  digitalWrite(WATER_FILL_PB_LED, digitalRead(WATER_OUTPUT));  
  
  
  // Track number of minutes water fill valve is opened each day
  if( isNewMinute() && digitalRead(WATER_OUTPUT) == HIGH )
  { waterAddedToday++; }

  // Sample pre-filter pressure for maximum pressure calculation
  if ( (long)(millis() - statsSampleTimer) > 0 && digitalRead(PUMP_OUTPUT) == HIGH )
  {
    // clear stats after 30 minutes
    if ( preFilterPressureStats.count() > ( 1800000UL / SAMPLE_TIME ) )
    { preFilterPressureStats.clear(); }
    
    preFilterPressureStats.add(pressure[PRE_FILTER_PRESSURE]);
    
    // If there are over 7 samples and pressure is in a normal range
    // Then set the nominal pre-filter pressure
    if ( preFilterPressureStats.count() >= 8 && pressure[PRE_FILTER_PRESSURE] > 14.0 )
    { lowPreFilterPressureThreshold = preFilterPressureStats.maximum() - 3.0; }
    
    statsSampleTimer = millis() + SAMPLE_TIME;
  }
  
  // If low pressure is detected, increase counter, only if water fill valve is closed
  if( pressure[PRE_FILTER_PRESSURE] < lowPreFilterPressureThreshold &&
      presFluctResetFlag == false &&
      digitalRead(WATER_OUTPUT) == LOW &&
      digitalRead(PUMP_OUTPUT) == HIGH )
  {
    presFluctCounter++;
    presFluctResetFlag = true;
  }
  
  // Reset flag for low pressure counter once pre-filter pressure increases
  if( pressure[PRE_FILTER_PRESSURE] > (lowPreFilterPressureThreshold + 3.0) && presFluctResetFlag == true )
  { presFluctResetFlag = false; }
  
  // If low pressure counter >= 20, and there is water fill pressure, then add 2x the pushbutton amount (30 min)
  // if water fill had been on for MAX_WATER_FILL_MINUTES minutes or less.
  // After that, don't add water and don't reset low pressure counter.
  if( presFluctCounter >= 20 &&
     digitalRead(WATER_OUTPUT) == LOW &&
     waterAddedToday < MAX_WATER_FILL_MINUTES &&
     pressure[WATER_FILL_PRESSURE] > WATER_FILL_PRESS_THRESH )
  {
    WaterFillTimer = millis() +  WATER_FILL_15_MIN;  // Add water
    presFluctCounter = 0;  // Reset low pressure counter; counter won't count up when water fill valve is on
    presFluctResetFlag = false;  // set to false so counter has chance to start again. If pressure never gets above 15, it won't reset
  }
  
  // Check for constant low pressure, not pressure fluctuations
  // If pressure is low and low pressure timer has not started yet and controller is in auto mode, then start lower pressure timer
  if( pressure[PRE_FILTER_PRESSURE] < 11 &&
     digitalRead(PUMP_OUTPUT) == HIGH &&
     digitalRead(PUMP_INPUT_AUTO) == PUMP_SWITCH_ON &&
     isLowPressTimerRunning == false ) 
  {
    lowPressTimer = millis() +  300000UL;  // start low pressure timer for 5 minutes (300k mS)
    isLowPressTimerRunning = true;  // set flag so this lowPressTimer isn't updated again
    Serial.println(F("Low pressure timer started"));
  }

  // If pressure returns to normal or pump is off, reset low pressure timer flag
  if( pressure[PRE_FILTER_PRESSURE] > (lowPreFilterPressureThreshold + 2.0) || digitalRead(PUMP_OUTPUT) == LOW )
  { isLowPressTimerRunning = false; }
  
  // Shutdown pump if amps are too high
  // Need to reboot Arduino to restart
  if( PumpAmps > 20 && EmergencyShutdown == EVERYTHING_OK )
  {
    EmergencyShutdown = HIGH_AMPS;
    poolStatus = STATUS_EMERGENCY_HI_AMPS;
  }
  
  // Shutdown pump if motor temp is too high
  // Need to reboot Arduino to restart
  if( temperature[PUMP_TEMP] > 180 && EmergencyShutdown == EVERYTHING_OK )
  {
    EmergencyShutdown = HIGH_PUMP_TEMP;
    poolStatus = STATUS_EMERGENCY_HI_PUMP_TEMP;
  }

  // Shutdown pump if low pressure counter goes to high
  // Counter will reset each time it reaches 20, and water is added.  But if there is not water fill pressure or
  // after MAX_WATER_FILL_MINUTES minutes of water is added, it will not reset, but keep qoing up.
  // Or if water to the water fill line is off, then pressue will be low and water won't be added
  if( presFluctCounter > 25 && EmergencyShutdown == EVERYTHING_OK )
  {
    EmergencyShutdown = LOW_PRESSURE;
    poolStatus = STATUS_EMERGENCY_LOW_PRESS_FLUCT;
  }
  
  // If pressure has been low for 5 minutes straight, set alarm status so shutdown pump  srgg
  if( (long)(millis() - lowPressTimer) > 0 && isLowPressTimerRunning && EmergencyShutdown == EVERYTHING_OK )
  {
    EmergencyShutdown = LOW_PRESSURE;
    poolStatus = STATUS_EMERGENCY_LOW_PRESS_COUNT;
  }

  // Turn pump on if:
  //   It's daytome and pump switch is in auto
  //   Or Pump switch is in manual mode
  bool isDayTime = (poolTime >= PUMP_ON_TIME  &&  poolTime <= PUMP_OFF_TIME);
  bool pumpSwitchAutoMode = (digitalRead(PUMP_INPUT_AUTO) == PUMP_SWITCH_ON);
  bool pumpSwitchManualMode = (digitalRead(PUMP_INPUT_MAN) == PUMP_SWITCH_ON);
  if( (isDayTime && pumpSwitchAutoMode && EmergencyShutdown == EVERYTHING_OK ) || pumpSwitchManualMode )
  { // Turn Pump On
    
    // See if pump is turning on now - oneshot
    if (digitalRead(PUMP_OUTPUT) == LOW )
    { statsSampleTimer = millis() + 30000UL; } // Initialize timer so pre-filter pressure samples start 30 seconds after pump comes on
    
    digitalWrite(PUMP_OUTPUT, HIGH);
    if( poolStatus <= STATUS_PUMP_SWITCH_OFF )
     { poolStatus = STATUS_PUMP_ON; }   // set pool status to Pump On, don't override higher status codes
  }
  else
  { // Turn Pump off
    digitalWrite(PUMP_OUTPUT, LOW);
    if( poolStatus <= STATUS_PUMP_SWITCH_OFF )
     { poolStatus = STATUS_PUMP_OFF; }  // set pool status to Pump Off, don't override higher status codes
  }
  
  // See if pump on/off switch is in off position, if so, set poolStatus, reset EmergencyShutdown, and reset pressure fluctuations
  if( !pumpSwitchAutoMode && !pumpSwitchManualMode )  // pump switch is off if it's in neither Auto or On modes
  {
    poolStatus = STATUS_PUMP_SWITCH_OFF;
    EmergencyShutdown = EVERYTHING_OK;
    presFluctCounter = 0;  // Reset low pressure counter; counter won't count up when water fill valve is on
    presFluctResetFlag = false;  // set to false so counter has chance to start again. If pressure never gets above 15, it won't reset
    preFilterPressureStats.clear();  // clear pre-filter pressure stats
    lowPreFilterPressureThreshold = 15;  // set default threshold
  }
  
  // Read Sensors
  // values are smoothed with low pass filter
  if ( (long)(millis() - SensorTimer) >= 0 )
  { // Read Sensors
    SensorTimer = millis() + SENSOR_READ_INTERVAL;  // every 250 mS

    // Read temperature sensors and use low pass filter to smooth
    float Smoothing = 0.3;  // smaller gives more smoothing, range 0 to 1.  1 is no smoothing
    waterTempSensors.requestTemperatures();   // Send the command to get temperatures
    for (byte i = 0; i < 3; i++)
    { temperature[i] = (Smoothing * waterTempSensors.getTempF(&tempSensors[i][0])) + ((1.0 - Smoothing) * temperature[i]); }

    // Read pressure sensors and use low pass filter to smooth
    float newPressure;
    newPressure = 0.0343 * (float) analogRead(PRESSURE1_PIN) - 5.9077;  // 12.5 psi = 500 ADC, 0 PSI = 193 ADC, calc seems to be about 1.5 PSI Low
    if (newPressure < 0.7)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    
    pressure[PRE_FILTER_PRESSURE] = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[PRE_FILTER_PRESSURE]);
    
    newPressure = 0.0359 * (float) analogRead(PRESSURE2_PIN) - 6.2548;
    if (newPressure < 0.8)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    
    pressure[POST_FILTER_PRESSURE] = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[POST_FILTER_PRESSURE]);


    // Water fill pressure could use two different sensors, 0-30 PSI or 0-100 PSI
    if ( PRESSURE_TRANSDUCER_TYPE == 30 )
    { newPressure = 0.0359  * (float) analogRead(WATER_FILL_PRESSURE_PIN) - 6.2548; } // using 0-30 PSI sensor
    else
    { newPressure = 0.12225 * (float) analogRead(WATER_FILL_PRESSURE_PIN) - 25.061; } // using 0-100 PSI sensor
    
    if (newPressure < 0.7)  // if pressure is near zero, set to zero
    { newPressure = 0.0; }
    
    pressure[WATER_FILL_PRESSURE]  = (Smoothing * newPressure) + ((1.0 - Smoothing) *  pressure[WATER_FILL_PRESSURE]);
    
    // Read pump amps and use low pass filter
    uint16_t pumpAmpsAnalog = analogRead(PUMP_AMPS_PIN);
    if ( pumpAmpsAnalog < 10 )
    { pumpAmpsAnalog = 0; }
    PumpAmps = (Smoothing * (float) pumpAmpsAnalog * 0.0185 ) + ((1.0 - Smoothing) *  PumpAmps);
    
    
    // Request I2C data from panStamp, will return water level sensor info
    getI2CData();

    // -----------------------------------------------
    // Validate sensor data and set sensorStatusbyte
    // -----------------------------------------------
    if (temperature[PRE_HEAT_TEMP] < 50 || temperature[PRE_HEAT_TEMP] > 180)
    { sensorStatusByte  &= ~(1 << 0); }  // Invalid pre-heat temperature, something wrong with sensor
    else
    { sensorStatusByte |= 1 << 0;}  // pre-heat temperature within acceptable range
    
    if (temperature[POST_HEAT_TEMP] < 50 || temperature[POST_HEAT_TEMP] > 180)
    { sensorStatusByte  &= ~(1 << 1); }  // Invalid post-heat temperature, something wrong with sensor
    else
    { sensorStatusByte |= 1 << 1; }  // post-heat temperature within acceptable range
    
    if (temperature[PUMP_TEMP] < 40 || temperature[PUMP_TEMP] > 300)
    { sensorStatusByte  &= ~(1 << 2); }  // Invalid temp range, something wrong with sensor
    else
    { sensorStatusByte |= 1 << 2; }  // valid temp range
    
    // Pre filter pressure
    // If pump is on and post-filter pressure is okay (>5), but higher then pre-filter pressure, then there is a problem with pre-filter
    if (digitalRead(PUMP_OUTPUT) == HIGH && pressure[POST_FILTER_PRESSURE] > 5 && pressure[POST_FILTER_PRESSURE] > pressure[PRE_FILTER_PRESSURE])
    { sensorStatusByte  &= ~(1 << 3); }  // Invalid pre-filter pressure
    // If pump is off, but pre-filter pressure is still high, then pressure sensor might have gone bad
    else if ( digitalRead(PUMP_OUTPUT) == LOW  && pressure[POST_FILTER_PRESSURE] > 25 && PumpAmps < 2 )
    { sensorStatusByte  &= ~(1 << 3); }  // Bad sensor
    else
    { sensorStatusByte |= 1 << 3; }  // pre-filter pressure okay
    
    // see if sensor is working, ADC value should never be below about 200
    if (analogRead(PRESSURE1_PIN) < 100 )
    { sensorStatusByte  &= ~(1 << 3); }  // Problem with sensor

    // Post filter pressure
    // If pump is on and pre-filter pressure is okay, but post filter pressure is low, then there is problem with sensor
    // Note - when discharging water to waste, this will cause post-filter pressure to show a problem
    if (digitalRead(PUMP_OUTPUT) == HIGH && pressure[PRE_FILTER_PRESSURE] > 10 && pressure[POST_FILTER_PRESSURE] < 4)
    { sensorStatusByte  &= ~(1 << 4); }  // Invalid pressure
    else
    { sensorStatusByte |= 1 << 4; }  // valid pressure range

    // see if sensor is working, ADC value should neve be below about 200
    if (analogRead(PRESSURE2_PIN) < 100 )
    { sensorStatusByte  &= ~(1 << 4); }  // Problem with sensor,

    // water fill pressure
    // Sensor outputs 1-5 volts, so min ADC should be about 200, so if it's less then 100, there is definitely a problem
    if(analogRead(WATER_FILL_PRESSURE_PIN) < 100)
    { sensorStatusByte  &= ~(1 << 5); } // Sensor failed (0)
    else
    { sensorStatusByte |= 1 << 5; }     // sensor ok (1)
    
    // Pump Amps
    // if pump is on but amps are low, there is a problem with sensor (or pump)
    if (digitalRead(PUMP_OUTPUT) == HIGH && PumpAmps < 4 )
    { sensorStatusByte  &= ~(1 << 6); }  // Invalid pump amps
    else
    { sensorStatusByte |= 1 << 6; }     // pump amps okay
    
    // Note: Water level sensor is set in getI2CData()

    setIoStatusByte();  // set bits in ioStatusByte
    
  }  // finished reading sensors
  
  // Send data to inside Arduino 
  if((long)(millis() - TX_Timer) >= 0 )
  {
    TX_Timer = millis() + TX_INTERVAL;  // every 2 secoonds
    sendXbeeData();   // Transmit data to inside Xbee
    printDebugFunction(isLowPressTimerRunning, lowPreFilterPressureThreshold); // use to debug sketch
  }

#ifdef ENABLE_OLED
  // Update OLED display
  if ( (long)(millis() - displayTimer) > 0 )
  {
    updateDisplay();
    displayTimer = millis() + 5000; 
  }
#endif

  // Reset minutes of water added today counter and low pressure counter at midnight
  if ( isNewDay() )
  {
    waterAddedToday =  0;
    presFluctCounter = 0;
    if ( poolStatus == STATUS_EMERGENCY_LOW_PRESS_COUNT )  // reset low pressure alarm
    {
      if ( pumpSwitchAutoMode )
      { poolStatus = STATUS_PUMP_OFF; }
      else if ( pumpSwitchManualMode )
      { poolStatus = STATUS_PUMP_ON; }
      EmergencyShutdown = EVERYTHING_OK;
      presFluctCounter = 0;
      presFluctResetFlag = false;
      isLowPressTimerRunning = false;  // set flag so this lowPressTimer isn't updated again
    }
  }
  
} // loop()


// set the bits in ioStatusbyte
void setIoStatusByte()
{
  // set bits in ioStatusbyte
  // shows input value for each I/O
  if(digitalRead(PUMP_OUTPUT) == HIGH)
   { ioStatusByte |= 1 << 0; }     // pump is on
  else
   { ioStatusByte &= ~(1 << 0); }  // pump is off
  
  if(digitalRead(PUMP_INPUT_AUTO) == LOW)
   { ioStatusByte |= 1 << 1; }     // 3-posistion switch is in Auto mode
  else
   { ioStatusByte &= ~(1 << 1); }  // 3-posistion switch not in Auto mode
  
  if(digitalRead(PUMP_INPUT_MAN) == LOW)
   { ioStatusByte |= 1 << 2; }     // 3-posistion switch is in manual mode
  else
   { ioStatusByte &= ~(1 << 2); }  // 3-posistion switch not in manual mode
  
  if(digitalRead(WATER_FILL_PB_LED) == HIGH)
   { ioStatusByte |= 1 << 3; }    // water fill LED is on
  else
   { ioStatusByte &= ~(1 << 3); }  // water fill LED is off
  
  if(digitalRead(WATER_FILL_PB) == LOW)
   { ioStatusByte |= 1 << 4; }     // water fill pushbutton is being pressed
  else
   { ioStatusByte &= ~(1 << 4); }  // water fill pushbutton is not pressed
  
  if(digitalRead(WATER_OUTPUT) == HIGH)
   { ioStatusByte |= 1 << 5; }     // water fill relay is on
  else
   { ioStatusByte &= ~(1 << 5); }  // water fill relay is off
  
  ioStatusByte &= ~(1 << 6);      // set heater relay status to off for now because it doesn't exist yet
  
  // Water level bit 7.  It gets set in getI2CData()
  
} // setIoStatusByte()


// send data to xbee inside house
bool sendXbeeData()
{
  // Put data in xbeeData array, multiple by 10 so you can get 1 decimal point, divide by 10 on the Rx side
  xbeeData[0] =  (int) (temperature[PRE_HEAT_TEMP]     * 10.0);
  xbeeData[1] =  (int) (temperature[POST_HEAT_TEMP]    * 10.0);
  xbeeData[2] =  (int) (temperature[PUMP_TEMP]         * 10.0);
  xbeeData[3] =  (int) (PumpAmps                       * 10.0);
  xbeeData[4] =  (int) (pressure[PRE_FILTER_PRESSURE]  * 10.0);
  xbeeData[5] =  (int) (pressure[POST_FILTER_PRESSURE] * 10.0);
  xbeeData[6] =  (int) (pressure[WATER_FILL_PRESSURE]  * 10.0);
  xbeeData[7] =        (presFluctCounter               * 10);    // counts low pressure fluctuations, means water level is low or leaves in filter
  xbeeData[8] =  (int) (poolStatus)                    * 10;
  xbeeData[9] =  (int) (waterAddedToday)               * 10;    // Number of minutes fill water valve was open
  xbeeData[10] = (int) (isLidLevel)                    * 10;
  xbeeData[11] = (int) (poolTime                       * 10.0);
  xbeeData[12] =       (levelSensorMilliVolts          * 10);     // battery volts (in mV) for water level sensor
  xbeeData[13] = (int) (lowWaterLevel)                 * 10;
  xbeeData[14] = (int) (sensorStatusByte)              * 10;
  xbeeData[15] = (int) (ioStatusByte)                  * 10;

  // Transmit data
  // break down integers into two bytes and place in payload
  for(int i=0; i < NUM_DATA_PTS; i++)
  {
    xbeePayload[i*2]     = xbeeData[i] >> 8 & 0xff; // High byte - shift bits 8 places, 0xff masks off the upper 8 bits
    xbeePayload[(i*2)+1] = xbeeData[i] & 0xff;      // Low byte, just mask off the upper 8 bits
  }
  xbee.send(tx);
  
  // after sending a tx request, we expect a status response
  // wait up to 1 seconds for the status response
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
        return true;
      }
      else
      {
        // the remote XBee did not receive our packet. is it powered on?
        return false;
      }
    }
  }
  else if (xbee.getResponse().isError())
  {
    #ifdef PRINT_DEBUG
      Serial.print(F("Error reading packet.  Error code: "));
      Serial.println(xbee.getResponse().getErrorCode());
    #endif
    return false;
  }
  else
  {
    // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
    #ifdef PRINT_DEBUG
      Serial.println(F("XBee did not provide a timely Tx Status Response"));
    #endif
    return false;
  }  // Finished waiting for XBee packet

  return false;
} // sendXbeeData()


//============================================================================
// I2C Request data from panStamp slave
//============================================================================
bool getI2CData()
{
  bool gotI2CPacket = false;
  byte i=0;  // counts packets received from panStamp
  byte i2CData[I2C_PACKET_SIZE];  // don't use char data type
  
  Wire.requestFrom(ADDR_SLAVE_I2C, I2C_PACKET_SIZE);    // request data from I2C slave
  
  while(Wire.available())    // Wire.available() will return the number of bytes available to read
  {
    i2CData[i++] = Wire.read(); // receive a byte of data
    gotI2CPacket = true;
  }

  // calculate I2C checksum
  if( gotI2CPacket )
  {
    byte checksum = 0;
    for ( byte cs = 0; cs < I2C_PACKET_SIZE - 1; cs++ )
    { checksum += i2CData[cs]; }
    
    if ( checksum != i2CData[I2C_PACKET_SIZE - 1] )
    {
      #ifdef PRINT_DEBUG
//        Serial.print(F("I2C Checksum Failed "));
      #endif
      return false;   // checksum failed, exit function
    }
  }
  
  // If we got an I2C packet, then extract the data from the I2C packet
  if( gotI2CPacket )
  {
    // if panStamp TX is online then extract data, if not return offline codes
    if ( i2CData[1] == PANSTAMP_ONLINE )
    {
      // water level detector Tx is online
      lowWaterLevel = (waterLevel_t) i2CData[2];   // Tx sets water as low level if it's continiously low for 2 minutes
      if( lowWaterLevel == LEVEL_SENSOR_OFFLINE )
      { sensorStatusByte  &= ~(1 << 7); } // water level sensor is offline
      else
      { sensorStatusByte |= 1 << 7; }     // water level sensor is online
        
      // set ioStatusbyte bit for real time water level
      // 0 = water okay, 1 = low water
      if( i2CData[3] == 1 )
      { ioStatusByte |= 1 << 7; }    // water level is low, set bit
      else
      { ioStatusByte &= ~(1 << 7); } // water level is okay, clear bit
      
      isLidLevel = i2CData[4];   // Is lid level
      
      // Battery voltage
      levelSensorMilliVolts = i2CData[11] << 8;
      levelSensorMilliVolts |= i2CData[12];
    }
    else
    { // water level detector TX is offline
      sensorStatusByte  &= ~(1 << 7);  // clear water level sensor bit to indicate sensor is offline
      lowWaterLevel = LEVEL_SENSOR_OFFLINE;
      levelSensorMilliVolts = 0;  // if sensor is offline, battery voltage can't be returned, so set it to zero
    }
    return true;
  }
  else
  { return false; } // No Packet received
  
} // end getI2CData()

#ifdef ENABLE_OLED
//============================================================================
// Rotate through the different screens
//============================================================================
void updateDisplay()
{
 
 static byte currentlyDisplaying = 0; // keeps track of what is now displayed on the OLED and rotates through screens
 display.clearDisplay();  
 display.setCursor(0,0);

 switch (currentlyDisplaying)
 {
   case 0:  // Display Temperature
     display.print(F("Pre-heat temp  "));
     display.println((int)temperature[PRE_HEAT_TEMP]);
     display.print(F("Post-heat temp "));
     display.println((int)temperature[POST_HEAT_TEMP]);
     display.print(F("Pump temp     "));
     display.print((int)temperature[PRE_HEAT_TEMP]);
     currentlyDisplaying = 1; 
     break;

   case 1: // Display pressure
     display.print(F("Pre-filter "));
     display.print(pressure[PRE_FILTER_PRESSURE],1);
     display.println(" PSI");
     display.print(F("Post-filtr "));
     display.print(pressure[POST_FILTER_PRESSURE],1);
     display.println(" PSI");
     display.print(F("Water fill "));
     display.print(pressure[WATER_FILL_PRESSURE],1);
     display.print(" PSI");
     currentlyDisplaying = 2;
     break;
  case 2: // display level sensor data
     display.print(F("Water level "));
     switch (lowWaterLevel)
     {
       case WATER_LEVEL_OK:
         display.println(F("ok"));
         break;
       case LOW_WATER:
         display.println(F("low"));
         break;
       case LEVEL_SENSOR_OFFLINE:
         display.println(F("offliine"));
         break;
     }

     display.print(F("Lid is ")); 
     if ( !isLidLevel )
     { display.print(F("not")); }
      display.println(F(" level"));
     
     display.print(F("Battery "));
     display.print(levelSensorMilliVolts);
     display.println(F(" mV"));
     currentlyDisplaying = 3;
     break;

   case 3:  // display water fill minutes today
     display.print(F("Water added ")); 
     display.print(waterAddedToday);
     display.println(F(" min"));
     currentlyDisplaying = 0;
     break;   
 } // end switch
 
 display.display();

} // end updateDisplay()
#endif

//============================================================================
// Return true if it's a new minutes
//============================================================================
bool isNewMinute()
{
  DateTime now = RTC.now();  // Gets the current time
  static byte prevMinute = now.minute();
  byte thisMinute = now.minute();
  
  if ( thisMinute == prevMinute )
  { return false;}
  else
  {
    prevMinute = thisMinute;
    return true;
  }
  
} // isNewMinute()


//============================================================================
// Return true if it's a new minutes
//============================================================================
bool isNewDay()
{
  DateTime now = RTC.now();  // Gets the current time
  static byte prevDay = now.day();
  byte thisDay = now.day();
  
  if ( thisDay == prevDay )
  { return false;}
  else
  {
    prevDay = thisDay;
    return true;
  }
  
} // isNewDay()

//============================================================================
// Prints out lots of pool info
// status byte, IO byte, low water, Lid level, Heat2, Fill Pres, P1, Low P tmr, P2, stat, ver, press count, pres threshold, pres max, pres stdev
//============================================================================
void printDebugFunction(bool lowPressTmr, float lowPresThreshold)
{
  static byte print_header = 20;
  
  if ( print_header < 20 )
  { print_header++; }
  else
  {
   Serial.println(F("stat\t\tIO\t\tlow H20\tlid\tHeat2\tFill\tP1\tPresTmr\tP2\tstatus\tver\t\samples\tthresh\tp-max\tstdev"));
   print_header = 0; 
  }
  
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(sensorStatusByte, j)); } // prints leading zeros in binary number
  Serial.print("\t");
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(ioStatusByte, j)); }  // print leading zeros
  Serial.print("\t");
  Serial.print(lowWaterLevel);  // low water for 2 minutes
  Serial.print("\t");
  Serial.print(isLidLevel);
  Serial.print("\t");
  Serial.print(temperature[POST_HEAT_TEMP]);
  Serial.print("\t");
  Serial.print(pressure[WATER_FILL_PRESSURE]);
  Serial.print("\t");
  Serial.print(pressure[PRE_FILTER_PRESSURE]);
  Serial.print("\t");
  Serial.print(lowPressTmr);
  Serial.print("\t");
  Serial.print(pressure[POST_FILTER_PRESSURE]); 
  Serial.print("\t");
  Serial.print(poolStatus);  // Pool statis ID
  Serial.print("\t");
  Serial.print(VERSION);  // sketch version
  
  // Pre-filter pressure stat
  Serial.print("\t");
  Serial.print(preFilterPressureStats.count());  // sample count
  Serial.print("\t");
  Serial.print(lowPresThreshold);  // Used to count pressure fluctuations.  It's set to 3PSI below max avg pressure
  Serial.print("\t");
  Serial.print(preFilterPressureStats.maximum());  // Max pressure, resets every 30 minutes
  Serial.print("\t");
  Serial.print(preFilterPressureStats.pop_stdev(),4);  // standard deviation of pre filter pressure
  
  Serial.println();
  
}  //printDebugFunction()


