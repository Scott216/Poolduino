/*
Xively Feed: http://xively.com/feeds/65673
Xbee Shield switch: to upload sketch, slide switch away from edge

Hardware: 
Arduino Mega
Xbee Series 1

To Do:
Change time so it's EST
I get a needs water alert if water filling after 15 minutes is not enough.  Change this so I only get alert if it needs water and can't deliver.
  I think this is sending the alert in the short period of time where it finished the 1st 15 minutes, but hasn't started the 2nd
If Xbee is offline, data for all streams still gets sent to Xively.  To resolve this, I think I need to use a different library.  Could get rid of library
 and do it the way it's done in this Adafruit exampele: http://git.io/vmCST
 I think xbee communication is affected by humidity
 
SD Card info: 
Format FAT 16
File name format: 8.3
Data is not saved until you use flush() or close()
Etherenet shield uses pin 4 for SD Card
SD.begin(4); // initialize SD Card

On the Mega, the hardware SS pin, 53, is not used to select the W5100, but it must be kept as an output or the SPI interface won't work.



Change Log
V1.50  06/23/14 - Added missing resets to some of the twitter flags (tf_).  Got rid of multiple alerts at 11PM.  Added functions for sensor status.  Don't upload streams to xively if sensor is bad or data is invalid.
v1.51  06/23/14 -  If sensor isn't working, sent n/a to SD card instead of last known value
v1.52  06/25/14 - Added bitly url to tweet
v1.53  06/26/14 - Fixed char overflow, removed pooltime from sendTweet() function.  When alarm is triggered, record will be saved to SD card
                  Changed Xbee timeout from 5 min to 30 sec
v1.54  06/28/14 - Changed xbee timeout, was getting too many alerts.  Have one timeout for Twitter message, another a shorter timeout for xively and SD card
                  temporarily logging low water level to SD card
v1.55  06/30/14 - reading xbee RSSI signal strength and saving to SC card along with xbee retries.  Prevent Sensor erros on bootup              
v1.56  07/02/14 - Added condition to check for Leonardo and to use Serial1 instead of Serial for Xbee so it would work with v0.5 of the xbee library
                  This sketch uses Mega, so it's not really necessary, but I wanted to be consistent with the other sketch
                  Removed check for Leonardo condition above.
v1.57  07/10/14 - Added PROGMEM statement to get rid of compiler warnigs
v1.58  07/30/14 - added column in log file for low water level.  Modified twitter alarms so tweet would only go out if 2 alarms were received a couple seconds apart.  Should help with false positives
v1.59  08/17/14 - Changed some heading in log file. Added new Xbee shield that uses Serial1 (pins D18-19 on Mega). Added Is lid level to xbee data
v1.60  09/05/14 - Add alerts if water is low and can't fill anymore.  Removed enum PoolDataIndex - it wasn't being used properly
                  Fixed bug in ReadXBeeData() where it wasn't reading last byte (ioStatusByte) - introduced when testing out checksum
v1.61  09/09/14 - Limit Xbee communication failure tweets to 4/day
v1.62  09/14/14 - Replace Xively failure feed with seconds since last xbee data received
v1.63  09/14/14 - Added LEDs that are on when Tx to Xively is okay and Rx from Xbee is okay.
v1.64  09/21/14 - Limit tweet alerts for level sensor trouble to 4
v1.65  09/25/14 - Forced pump amps to zero if it's below 1/4 amp
v1.66  05/21/15 - Added sketch version to Tweet message.  Only send volts to Xively if volts > 2000mV, this will stop all the zero volts from getting plotted
v1.67  06/21/15 - Change low water level Tweet to only send out if water  has been on for more then 40 minutes.  Also sends out how many minutes it's been on so far for the day.
                  Fixed tweet suffix.  It wasn't properly adding the sketch version. Added Tweet if water temp is >= 85
v1.68  07/05/15 - Changed NTP time server
v1.69  07/06/15 - Cast float to int for water minute tweets 
v1.70  07/08/15 - Fixed water fill alert so it only sends text if it's over 40 minutes.  I mistakenly had it less then 40 min
v1.71  07/11/15 - Changed NTP to use Time_NTP.h library I made.  Added 30 minute delay to Water Level sensor trouble alert.  Added g_ prefix to global variables
v1.72  07/15/15 - Fixed Xbee alert - it should go out after 10 minutes without xbee communication, but it was really set for 1 minute.
v1.73  07/26/15 - Added adjustment for EST time
v1.74  06/14/16 - v1.73 was causing reboots, so I took the EST adjustment out. 
*/

#define VERSION "v1.74"
#define PRINT_DEBUG     // Comment out to turn off serial printing

#include <Ethernet.h>        // Library for Arduino ethernet shield http://arduino.cc/en/Reference/Ethernet
#include <HttpClient.h>      // http://github.com/amcewen/HttpClient/blob/master/HttpClient.h
#include <Time.h>            // http://playground.arduino.cc/Code/time
#include <SPI.h>             // http://arduino.cc/en/Reference/SPI
#include <SD.h>              // Micro SD Card. http://arduino.cc/en/Reference/SD
#include "Xively.h"          // http://github.com/xively/xively_arduino
#include "Twitter.h"         // http://arduino.cc/playground/Code/TwitterLibrary, get token from token at http://arduino-tweet.appspot.com/
#include "XBee.h"            // http://code.google.com/p/xbee-arduino/
#include "Tokens.h"          // Tokens for Xively and twitter
#include "Pool_Controller_Inside_Library.h"    // Include application, user and local libraries
#include "Time_NTP.h"        // My Time library

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// I/O
const byte SD_CARD_SS =   4;
const byte SPI_ENABLE  = 53;  // Must be defined as an output for SPI to work properly, see: http://arduino.cc/en/Reference/SDbegin
// D10 is reserved for Etherent SS

// Index positions for PoolData[] array
const uint8_t P_TEMP1 =              0;  // Temperature before heater
const uint8_t P_TEMP2 =              1;  // Temperature after
const uint8_t P_TEMP_PUMP =          2;  // Pump housing temperature
const uint8_t P_PUMP_AMPS =          3;  // Amps pump is using
const uint8_t P_PRESSURE1 =          4;  // Pressure before filter
const uint8_t P_PRESSURE2 =          5;  // Pressure after filter
const uint8_t P_PRESSURE3 =          6;  // Pressure of water fill line
const uint8_t P_LOW_PRES_CNT =       7;  // Counts times pressure was low
const uint8_t P_CONTROLLER_STATUS =  8;  // Controller status 0:pump off, 1:pump on, 2:switch in off position, 3:water filling, 4:Fluctuations, 5:Low Pressure, 6:Hi amps, 7:hi temp, 8:remote shutdown
const uint8_t P_WATER_FILL_MINUTES = 9;  // Minutes water fill valve was open today
const uint8_t P_LID_IS_LEVEL =      10;  // Is lid level on water level sensor
const uint8_t P_POOL_TIME =         11;  // Pool time from RTC, 2:45 PM = 14.75
const uint8_t P_WATER_LVL_BATT =    12;  // Water level battery voltage
const uint8_t P_LOW_WATER =         13;  // Low water sensor: 0 = level ok, 1 = low water, 2 = offline
const uint8_t P_SENSORSTATUSBYTE =  14;  // Sensor Inputs Status Byte: 1 if sensor is working properly, 0 of not
const uint8_t P_IOSTATUSBYTE =      15;  // Discrete I/O status byte: shows on/off state if I/O
const uint8_t NUM_POOL_DATA_PTS =   16;  // Number of data points in pool array xbee packet
const uint8_t SOMETHING_IS_WRONG =   4;  // If P_CONTROLLER_STATUS is 4 or higher, there was a shutdown


const uint8_t TX_LED_PIN = 5;  // LED to indicate upload to Xively is okay
const uint8_t RX_LED_PIN = 6;  // LED to indicate data from Xbee is okay


byte g_sensorStatusByte;          // Each bit determines if sensor is operating properly
byte g_ioStatusByte;              // Each bit shows input value of digital I/O

// Number of Xively Streams
#define NUM_XIVELY_STREAMS  16

const byte     STRLEN_MAX_TWEET =           75;   // Character array size for twitter message
const uint32_t XIVELY_UPDATE_INTERVAL =  15000;   // Xively upload interval (mS)
const uint32_t XIVELY_UPDATE_TIMEOUT = 1800000;   // 30 minute timeout - if there are no successful updates in 30 minutes, reboot
#define FEED_ID   65673                           // Xively Feed ID http://xively.com/feeds/65673
// #define FEED_ID 4663  // Test feed

uint32_t g_xively_uploadTimout_timer; // Timer to reboot if no successful uploads in 30 minutes
uint32_t g_xively_Upload_Timer;       // Timer for uploading to Xively

const int BUFFERSIZE = 30;
char g_bufferValue[BUFFERSIZE]; // enough space to store the string we're going to send to Xively
const byte STATUSBUFLEN = 26;   // character buffer length for controller status text

// format is XivelyDatastream(stream name text, length of stream name, variable type: DATASTREAM_FLOAT, DATASTREAM_INT)
// for text format is XivelyDatastream(stream name text, lenght of stream name, variable type: DATASTREAM_BUFFER, data as text, length of data as text )
XivelyDatastream datastreams[] =
{
  XivelyDatastream( "0", 1, DATASTREAM_FLOAT),  // Pressure before filter
  XivelyDatastream( "1", 1, DATASTREAM_FLOAT),  // Pressure after filter
  XivelyDatastream( "2", 1, DATASTREAM_FLOAT),  // Water Fill Pressure
  XivelyDatastream( "3", 1, DATASTREAM_FLOAT),  // Temp before heater
  XivelyDatastream( "4", 1, DATASTREAM_FLOAT),  // Temp after heater
  XivelyDatastream( "5", 1, DATASTREAM_INT),    // Heater On/Off Status
  XivelyDatastream( "6", 1, DATASTREAM_FLOAT),  // Pump housing temperature
  XivelyDatastream( "7", 1, DATASTREAM_FLOAT),  // Pump amps
  XivelyDatastream( "8", 1, DATASTREAM_INT),    // Low Pressure Count
  XivelyDatastream( "9", 1, DATASTREAM_INT),    // Minutes water fill valve was otpen today
  XivelyDatastream("10", 2, DATASTREAM_BUFFER, g_bufferValue, BUFFERSIZE), // Status of controller.  Only stream where text is sent instead of a number
  XivelyDatastream("11", 2, DATASTREAM_INT),    // Xively upload successes
  XivelyDatastream("12", 2, DATASTREAM_INT),    // Seconds since last Xbee data
  XivelyDatastream("13", 2, DATASTREAM_INT),    // Status of controller - number
  XivelyDatastream("14", 2, DATASTREAM_INT),    // Water level sensor
  XivelyDatastream("15", 2, DATASTREAM_FLOAT)   // battery volts for water level sensor
//  XivelyDatastream("16", 2, DATASTREAM_INT)     // Future - Xbee RSSI signal strengh 
//  XivelyDatastream("17", 2, DATASTREAM_INT)     // Future - XBee retries
};

// Wrap the datastreams into a feed
XivelyFeed feed(FEED_ID, datastreams, NUM_XIVELY_STREAMS);


// Ethernet Setup
EthernetClient client;
XivelyClient xivelyclient(client);
byte mac[] = { 0xCC, 0xAC, 0xBE, 0x21, 0x91, 0x43 };
uint8_t g_successes = 0;    // Xively upload success, will rollover at 255, but that's okay.  This makes is easy to see on Xively is things are running
uint8_t g_failures =  0;      // Xively upload failures


// Xbee Setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();

bool     g_gotNewData =      false;  // Flag to indicate that sketch has received new data from xbee
uint32_t g_xbeeLastRxTime =      0;  // Last time data was received from xbee
bool     g_xBeeTimeoutFlag = false;  // Flag to indicate no data from Xbee, used to keep warning from going off every 5 minutes
int8_t   g_xbeeSignal =          0;  // xbee RSSI Signal strength
uint8_t  g_xbeeRetries =         0;

const int CHIPSELECT = 4;  // Micro SD Card

// Twitter setup
Twitter twitter(TWITTER_TOKEN);


// Array to hold pool data received from outside controller
float g_poolData[NUM_POOL_DATA_PTS];


// Declare function prototypes
void PrintPoolData();
void software_Reset();
bool SendDataToXively();
bool ReadXBeeData(uint16_t *Tx_Id);
bool logDataToSdCard(char msgComment[]);
void checkAlarms();
int SendTweet(char msgTweet[]);
int freeRam(bool PrintRam);
void controllerStatus(char * txtStatus, int poolstatus);
bool isPreFltrPressSensorOk();
bool isPostFltrPressSensorOk();
bool isWaterFillPressSensorOk();
bool isPreHtrTempSensorOk();
bool isPostHtrTempSensorOk();
bool isPumpTempSensorOk();
bool isPumpAmpsSensorOk();
bool isWaterLevelSensorOk();

bool isWaterFillValveOpen();

//============================================================================
//============================================================================
void setup(void)
{
  Serial.begin(9600);

  #ifdef PRINT_DEBUG
    Serial.print(F("\nSetup pool controller inside "));
    Serial.println(VERSION);
  #endif

  // Xbee is using Serial1 on Mega pins D18 (Tx) D19 (Rx)
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
  
  // Initialize SPI pins
  pinMode(SPI_ENABLE, OUTPUT);
  pinMode(SD_CARD_SS, OUTPUT);
  digitalWrite(SD_CARD_SS, HIGH);  // disable microSD card interface while ethernet is starting

  // Initialize Ethernet
  Ethernet.begin(mac);
  Serial.print("IP (DHCP): ");
  Serial.println(Ethernet.localIP());

  // Setup LED pins and flash LEDs
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(RX_LED_PIN, OUTPUT);
  for (byte j = 0; j < 3; j++ )
  {
    digitalWrite(TX_LED_PIN, HIGH);
    digitalWrite(RX_LED_PIN, HIGH);
    delay(100);
    digitalWrite(TX_LED_PIN, LOW);
    digitalWrite(RX_LED_PIN, LOW);
    delay(100);
  }
  
  // Setup NTP time
  setSyncProvider(getNewNtpTime);
  
  //  Print the time
  #ifdef PRINT_DEBUG
    char timebuf[16];
    sprintf(timebuf, "Time: %02d:%02d:%02d", hour(),minute(),second());
    Serial.println(timebuf);
  #endif
  	
  // Initialize SD Card
  if ( SD.begin(CHIPSELECT) )
  #ifdef PRINT_DEBUG
    { Serial.println(F("SD card initialized")); }
    else
    { Serial.println(F("SD Card failed, or not present")); }
  #endif
  
  // Initialize global variables
  g_xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
  g_xively_Upload_Timer       = millis() + XIVELY_UPDATE_INTERVAL;
    
  g_xbeeLastRxTime = millis();  // initialize timer, didn't get xbee data yet, but don't want to generate an error on startup
  
  #ifdef PRINT_DEBUG
    Serial.println(F("End setup"));
    freeRam(true);
  #endif
  
} // end setup()


//=========================================================================================================
//============================================================================
void loop(void)
{
  uint16_t xbeeID;                      // ID of transimitting xbee
  char msgTweet[STRLEN_MAX_TWEET];      // Holds text for twitter message.  Should be big enough for message and timestamp
  static bool gotFistXbeeData = false;  // Needed at reboot to prevent false alarms for sensor status.  Set to true after first good xbee transmit 
  
  // Read XBee data
  // Try up to 30 times to read xbee data, after that give up and move on
  bool xbeeStat;
  byte xbeeFailCnt = 0;
  do
  {
    delay(250);
    xbeeStat = ReadXBeeData(&xbeeID);
    xbeeFailCnt++;
  } while (xbeeStat == false && xbeeFailCnt < 30 );
  g_xbeeRetries = xbeeFailCnt;
  
  // One shot flag to indicate we got first data from xbee, only resets on reboot
  if( xbeeStat) 
  { gotFistXbeeData = true; }
 
  // Send a Tweet for startup, do this after you read xbee data so you can append pooltime which avoids duplicate tweets
  static bool tweetstartup;
  if (tweetstartup == false && millis() > 20000UL)
  {
    strcpy(msgTweet, "Inside pool monitor has restarted.");
    SendTweet(msgTweet);
    tweetstartup = true;
  }
  
  // Check for Xbee timeout, 30 seconds
  if( (long)(millis() - g_xbeeLastRxTime) >= 30000L )
  { g_xBeeTimeoutFlag = true; }
  

  // Turn on LED if Xbee data is coming in okay
  digitalWrite(RX_LED_PIN, !g_xBeeTimeoutFlag);
  
    // Log water level sensor status
//srg    if ( xBeeTimeoutFlag == false && PoolData[P_LOW_WATER] == 1)
//    { logDataToSdCard(""); }

  // Upload data to Xively
  if ((long)(millis() - g_xively_Upload_Timer) >= 0)
  {
    g_xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL;  // Reset timer
    bool uploadStatus = SendDataToXively();  // Send Data to Xively
    digitalWrite(TX_LED_PIN, uploadStatus);  // If upload to Xively is okay, turn on LED
    
    // Log data to SD card, if we are receiving data from outside controller
    char emptyString[] = "";
    if ( g_xBeeTimeoutFlag == false )
    { logDataToSdCard(emptyString); }
  }
  
  // Check inputs and send message (Tweet) if anything is wrong
  // Only do this if we have establisehd communication with Xbee at least once
  if ( gotFistXbeeData )
  { checkAlarms(); }
  
  // Reboot if no successful updates to Xively in 30 minutes
  if((long) (millis() - g_xively_uploadTimout_timer) >= 0)
  {
    strcpy(msgTweet, "Reboot: can't upload to Xively.");
    SendTweet(msgTweet);
    g_xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
    delay(1000);
    software_Reset();
  }
  
} // end loop()


// Check inputs and send out a tweet if anything is wrong
// Don't send tweet upon first alarm since sometimes they are false positive
// If alarm comees in, wait a couple seconds and if it comes in again, than send the tweet
void checkAlarms()
{

  enum alarmFlag{ NO_ALARM, ALARM_1, ALARM_2 };
  // Twitter Flags (tf_) used so Twitter messages are only sent once
  static alarmFlag tf_highPresDrop;       // High pressure across filter
  static alarmFlag tf_highPressure;       // Pressure before filter
  static alarmFlag tf_lowPressure;        // Low pressure
  static alarmFlag tf_highAmps;           // High pump amps
  static alarmFlag tf_highPumpTemp;       // High pump temperature
  static alarmFlag tf_emergencyShutdown;  // Emergency shutdown flag
  static alarmFlag tf_pumpOnAtNight;      // Pump on at night
  static alarmFlag tf_pumpOffInDay;       // Pump is off during the day
  static alarmFlag tf_waterFillOn;        // Water fill valve is on
  static alarmFlag tf_heaterIsOn;         // Heater is on
  static alarmFlag tf_waterIsHot;         // Water temp is high
  static alarmFlag tf_Xbee_Comm;          // Xbee communication, send alert if no comm
  static alarmFlag tf_cantAddWater;       // Need water, but water fill valve isn't open
  // flags for sensor status.
  static alarmFlag tf_preHeatTemperatureSensor;
  static alarmFlag tf_postHeatTemperatureSensor;
  static alarmFlag tf_pumpTemperatureSensor;
  static alarmFlag tf_preFilterPresureSensor;
  static alarmFlag tf_postFilterPressureSensor;
  static alarmFlag tf_waterFillPressureSensor;
  static alarmFlag tf_pumpAmpsSensor;
  static alarmFlag tf_waterLevelSensor;
  const uint16_t ALARM_DELAY = 2000;
  
  
  char msgAlarm[STRLEN_MAX_TWEET];    // Holds text for twitter message.  Should be big enough for message and timestamp

  // If Arduino recently restarted (last 10 seconds), set waterFillCntDnFlag to true so it
  // doesn't send a tweet if the water fill is on
  if(millis() < 10000UL)
  { tf_waterFillOn = ALARM_2; }
  
  
  // High pump amps
  if( g_poolData[P_PUMP_AMPS] >= 17.0 && tf_highAmps != ALARM_2 )
  {
    if( tf_highAmps == NO_ALARM )
    { 
      tf_highAmps = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    {
      sprintf(msgAlarm, "High pump amps: %d.", (int) g_poolData[P_PUMP_AMPS]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highAmps = ALARM_2;
    }
  }

  // Reset high amps flag
  if( g_poolData[P_PUMP_AMPS] < 10 )
  { tf_highAmps = NO_ALARM; }

  // High pump temperature
  if( g_poolData[P_TEMP_PUMP] >= 175.0 && tf_highPumpTemp != ALARM_2 )
  {
    if (tf_highPumpTemp == NO_ALARM )
    {
      tf_highPumpTemp = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      sprintf(msgAlarm, "High pump temp: %d.", (int) g_poolData[P_TEMP_PUMP]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highPumpTemp =  ALARM_2; 
    }
  }

  // Reset high pump temp flag
  if( g_poolData[P_TEMP_PUMP] < 110 )
  { tf_highPumpTemp = NO_ALARM; }
  
  // High pressure
  if( g_poolData[P_PRESSURE1] >= 40.0 && tf_highPressure != ALARM_2 )
  {
    if ( tf_highPressure == NO_ALARM )
    {
      tf_highPressure = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      sprintf(msgAlarm, "High pump pressure: %d.", (int) g_poolData[P_PRESSURE1]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highPressure = ALARM_2;
    }
  }

  // Reset high pressure flag
  if( g_poolData[P_PRESSURE1] < 110 )
  { tf_highPressure = NO_ALARM; }

  // Check pressure drop across filter
  if( (g_poolData[P_PRESSURE1] > 25.0) && ((g_poolData[P_PRESSURE1] - g_poolData[P_PRESSURE2]) > 10.0) && (tf_highPresDrop != ALARM_2) )
  {
    if ( tf_highPresDrop == NO_ALARM )
    {
      tf_highPresDrop = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "High filter pressure.");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highPresDrop = ALARM_2;
    }
  }

  // Reset high pressure drop Flag
  if( g_poolData[P_PRESSURE1] < 2 )
  { tf_highPresDrop = NO_ALARM; }
  
  // Check low pressure Counter
  if( g_poolData[P_LOW_PRES_CNT] >= 17.0 && tf_lowPressure != ALARM_2  )
  {
    if( tf_lowPressure == NO_ALARM )
    {
      tf_lowPressure = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Low pressure fluctuations at pool pump.");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_lowPressure = ALARM_2;
    }
  }

  // Reset low pressure counter flag
  if( g_poolData[P_LOW_PRES_CNT] == 0 )
  { tf_lowPressure = NO_ALARM; }

  
  // Need water but water fill valve isn't open
  // Water fill valve won't open if max water fill minutes was reached or there is not water pressure
  if ( g_poolData[P_LOW_WATER] == 1 && isWaterFillValveOpen() == false && tf_cantAddWater != ALARM_2 )
  {
    if( tf_cantAddWater == NO_ALARM )
    {
      tf_cantAddWater = ALARM_1;
      delay(ALARM_DELAY);
    }
    else
    {
      strcpy(msgAlarm, "Need water");
      logDataToSdCard(msgAlarm);
      SendTweet(msgAlarm);
      tf_cantAddWater = ALARM_2;
    }
  }
  
  // Reset can't add water flag
  if( isWaterFillValveOpen() )
  { tf_cantAddWater = NO_ALARM; }
  
  // Check for emergency shutdown
  if( g_poolData[P_CONTROLLER_STATUS] >= SOMETHING_IS_WRONG && tf_emergencyShutdown != ALARM_2 )
  {
    if( tf_emergencyShutdown == NO_ALARM )
    {
      tf_emergencyShutdown = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      char txtStatus[STATUSBUFLEN];
      strcpy(msgAlarm, "Emergency Shutdown - ");
      controllerStatus(txtStatus,  g_poolData[P_CONTROLLER_STATUS]);
      strcat(msgAlarm, txtStatus);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      #ifdef PRINT_DEBUG
        Serial.println(msgAlarm);
      #endif
      tf_emergencyShutdown = ALARM_2;
    }
  }

  // Reset emergency shutdown flag
  if( g_poolData[P_CONTROLLER_STATUS] < SOMETHING_IS_WRONG )
  { tf_emergencyShutdown = NO_ALARM; }
  
  // Send Tweet if water fill has started and it's already filled water for 40 minutes or more
  // The pool seems to need water every day, so I only want a tweet if it needs a lot of water
  if( isWaterFillValveOpen() && tf_waterFillOn != ALARM_2 && g_poolData[P_WATER_FILL_MINUTES] >= 40.0 )
  {
    // Wait a couple seconds in case water fill button is pressed a couple times, then read XBee data again
    delay(2500);
    uint16_t xbeeID;
    ReadXBeeData(&xbeeID);
    sprintf(msgAlarm, "Water fill started. So far %d min.", (int) g_poolData[P_WATER_FILL_MINUTES] );
    logDataToSdCard(msgAlarm);
    SendTweet(msgAlarm);
    tf_waterFillOn = ALARM_2;  // flag so Tweet is only sent once
  }

  // Reset water fill timer flag
  if( isWaterFillValveOpen() == false )
  { tf_waterFillOn = NO_ALARM; }

  // Send Tweet if pump is running at night
  if( g_poolData[P_PUMP_AMPS] > 6.0 && g_poolData[P_POOL_TIME] >= 20.3 && tf_pumpOnAtNight != ALARM_2  )  // 8:30 PM
  {
    if( tf_pumpOnAtNight == NO_ALARM )
    {
      tf_pumpOnAtNight = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Dude, turn off the pool pump");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_pumpOnAtNight = ALARM_2;
    }
  }

  // Send Tweet if pump not running during the day
  if( g_poolData[P_PUMP_AMPS] < 2.0 && g_poolData[P_POOL_TIME] > 8.0 && g_poolData[P_POOL_TIME] < 17.0 && tf_pumpOffInDay != ALARM_2  )
  {
    if( tf_pumpOffInDay == NO_ALARM )
    {
      tf_pumpOffInDay = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "The pool pump is not running");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_pumpOffInDay = ALARM_2;
    }
  }

  // Send tweet when heater comes on.  Don't want this every time, just once a day, so reset at night
  // Determine if Pool heater is on by comparing temperatures
  if( ((g_poolData[P_TEMP2] - g_poolData[P_TEMP1]) > 5.0) && (g_poolData[P_PUMP_AMPS] > 5.0) && tf_heaterIsOn != ALARM_2 )
  {
    if( tf_heaterIsOn == NO_ALARM )
    {
      tf_heaterIsOn = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool heater is on");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_heaterIsOn = ALARM_2;
    }
  }

  // Send tweet when water temp is over 85.  Don't want this every time, just once a day, so reset at night
  if( g_poolData[P_TEMP1] >= 85.0 &&  tf_waterIsHot != ALARM_2 )
  {
    if( tf_waterIsHot == NO_ALARM )
    {
      tf_waterIsHot = ALARM_1;
      delay(ALARM_DELAY);
    }
    else
    {
      sprintf(msgAlarm, "Pool water temp is %d", (int)g_poolData[P_TEMP1]);
      logDataToSdCard(msgAlarm);
      SendTweet(msgAlarm);
      tf_waterIsHot = ALARM_2;
    }
  }
  
  
  
  // Send alert if xbee communication is lost for 10 mintues
  static byte tweetCounterXbee = 0; // don't send more then 4 xbee comm failures per day
  if( (long)(millis() - g_xbeeLastRxTime) > 6000000L  && tf_Xbee_Comm != ALARM_2 && tweetCounterXbee < 3 )
  {
    if( tf_Xbee_Comm == NO_ALARM )
    {
      tf_Xbee_Comm = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Lost xBee communication");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_Xbee_Comm = ALARM_2;
      tweetCounterXbee++;
    }
  }
  
  // Reset xbee communication flag
  if( g_gotNewData == true )
  { tf_Xbee_Comm = NO_ALARM; }


  // Send tweet if any sensors are having trouble
  
  // Check pre heater temp sensor
  if ( !isPreHtrTempSensorOk() && tf_preHeatTemperatureSensor != ALARM_2  )
  {
    if( tf_preHeatTemperatureSensor == NO_ALARM )
    {
      tf_preHeatTemperatureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool pre-heat temperature sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_preHeatTemperatureSensor = ALARM_2;
    }
  }

  // Reset pre heater temp flag
  if ( isPreHtrTempSensorOk() && tf_preHeatTemperatureSensor )
  { tf_preHeatTemperatureSensor = NO_ALARM; }

  // Check post heater temp sensor
  if ( !isPostHtrTempSensorOk() && tf_postHeatTemperatureSensor != ALARM_2 )
  {
    if( tf_postHeatTemperatureSensor == NO_ALARM )
    {
      tf_postHeatTemperatureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool post-heat temperature sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_postHeatTemperatureSensor = ALARM_2;
    }
  }

  // Reset post heater temp sensor
  if ( isPostHtrTempSensorOk() )
  { tf_postHeatTemperatureSensor = NO_ALARM; }

  // Check pump temp sensor
  if ( !isPumpTempSensorOk() && tf_pumpTemperatureSensor != ALARM_2 )
  {
    if( tf_pumpTemperatureSensor == NO_ALARM )
    {
      tf_pumpTemperatureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool pump temperature sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_pumpTemperatureSensor = ALARM_2;
    }
  }

  // Reset pump temp sensor flag
  if ( isPumpTempSensorOk() )
  { tf_pumpTemperatureSensor = NO_ALARM; }

  // Check pre filter pressure sensor
  if ( !isPreFltrPressSensorOk() && tf_preFilterPresureSensor != ALARM_2 )
  {
    if( tf_preFilterPresureSensor == NO_ALARM )
    {
      tf_preFilterPresureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool pre-filter pressure sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_preFilterPresureSensor = ALARM_2;
    }
  }

  // Reset pre filter pressure sensor flag
  if ( isPreFltrPressSensorOk() )
  { tf_preFilterPresureSensor = NO_ALARM; }

  // check post filter pressure sensor
  if ( !isPostFltrPressSensorOk() && tf_postFilterPressureSensor != ALARM_2 )
  {
    if( tf_postFilterPressureSensor == NO_ALARM )
    {
      tf_postFilterPressureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool post-filter pressure sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_postFilterPressureSensor = ALARM_2;
    }
  }

  // Reset post filter pressure sensor flag
  if ( isPostFltrPressSensorOk() )
  { tf_postFilterPressureSensor = NO_ALARM; }

  
  // Check water fill pressure sensor
  if ( !isWaterFillPressSensorOk() && tf_waterFillPressureSensor != ALARM_2 )
  {
    if( tf_waterFillPressureSensor == NO_ALARM )
    {
      tf_waterFillPressureSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool water fill pressure sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_waterFillPressureSensor = ALARM_2;
    }
  }

  // Reset water fill pressure sensor flag
  if ( isWaterFillPressSensorOk() )
  { tf_waterFillPressureSensor = NO_ALARM; }

  // Check pump amps sensor
  if ( !isPumpAmpsSensorOk() && tf_pumpAmpsSensor != ALARM_2 )
  {
    if( tf_pumpAmpsSensor == NO_ALARM )
    {
      tf_pumpAmpsSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool pump amps sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_pumpAmpsSensor = ALARM_2;
    }
  }
  
  // Reset pump amps sensor flag
  if ( isPumpAmpsSensorOk() )
  { tf_pumpAmpsSensor = NO_ALARM; }


  // check water level sensor
  // Don't send out alert unless sensor is offline for 30 minutes
  static byte tweetCounterLevelSensor = 0;
  static uint32_t levelSensorOnlineTimestamp = millis();
  if ( isWaterLevelSensorOk() )
  { levelSensorOnlineTimestamp = millis(); }
  bool isOfflineForLongTime =  ( (long)(millis() - levelSensorOnlineTimestamp) > 1800000UL );
    
  if ( tf_waterLevelSensor != ALARM_2 && tweetCounterLevelSensor < 3 && isOfflineForLongTime )
  {
    strcpy(msgAlarm, "Pool water level sensor trouble");
    logDataToSdCard(msgAlarm);
    SendTweet(msgAlarm);
    tf_waterLevelSensor = ALARM_2;
    tweetCounterLevelSensor++;
  }
  
  // Reset water Level sensor flag
  if ( isWaterLevelSensorOk() )
  { tf_waterLevelSensor = NO_ALARM;}

  // At 11:01 PM reset flags for:
  //   pump running at night
  //   Pump not running in day
  //   heater is on
  //   Xbee communication failure
  //   Level sensor problem
  if(hour() == 23 && minute() == 1 && second() < 10 )
  {
    // Reset twitter flags
    tf_pumpOnAtNight = NO_ALARM;
    tf_pumpOffInDay =  NO_ALARM;
    tf_heaterIsOn =    NO_ALARM;
    tf_waterIsHot =    NO_ALARM;
    tweetCounterXbee = 0;
    tweetCounterLevelSensor = 0;
    delay(10000); // delay 10 seconds to prevent multiple tweets from going out at 11 PM
  }

  // tf_pumpOffInDay flag can also be reset if pump it turned back on
  if(g_poolData[P_PUMP_AMPS] > 6 && ((g_sensorStatusByte >> 6) & 1) == 1)
  { tf_pumpOffInDay = NO_ALARM; }

} // end checkAlarms()


//=========================================================================================================
// Send twitter text, appends the time to the message to avoid twitter blocking duplicate messages
// bitly link to xively: bit.ly/1vapvvo
//=========================================================================================================
int SendTweet(char * txtTweet)
{
  
  g_xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL; // increase timer for Xively so it doesn't send right after Twitter

  logDataToSdCard(txtTweet); // update log file

  const uint8_t STRLEN_TIMESTAMP = 20;
  const uint8_t STRLEN_BITLY =     15;
  const uint8_t STRLEN_VERSION =    8;
  char tweetSuffix[STRLEN_TIMESTAMP + STRLEN_VERSION + STRLEN_BITLY];   // char array that contains sketch version, pool time & bitly URL
  sprintf(tweetSuffix, " Time: %d:%02d\n", hour(), minute());
  strcat(tweetSuffix, VERSION);
  strcat(tweetSuffix, "\nbit.ly/1vapvvo");
  if( strlen(txtTweet) <= (STRLEN_MAX_TWEET - strlen(tweetSuffix)) ) // Make sure message there is room in character array for the timestamp, version & bitly URL
  { strcat(txtTweet, tweetSuffix); }
  
  #ifdef PRINT_DEBUG
    Serial.println(txtTweet);
  #endif
  
  if (twitter.post(txtTweet))
  {
    // Specify &Serial to output received response to Serial.
    // If no output is required, you can just omit the argument, e.g.
    // int tweetStatus = twitter.wait();
    #ifdef PRINT_DEBUG
      int tweetStatus = twitter.wait(&Serial);
      Serial.println();
    #else
      int tweetStatus = twitter.wait();
    #endif
    if (tweetStatus == 200)
    {
      #ifdef PRINT_DEBUG
        Serial.println(F("Twitter OK."));
      #endif
      return 200;
    }
    else
    {
      #ifdef PRINT_DEBUG
        Serial.print(F("Twitter failed: code "));
        Serial.println(tweetStatus);
      #endif
      return tweetStatus;
    }
  }
  else
  {
    #ifdef PRINT_DEBUG
      Serial.println(F("Twitter connection failed."));
    #endif
    return 0;
  }
  
} // end SendTweet()


//=========================================================================================================
// Send data to Xively
// xivelyData array is the same as the poolData array in loop function
// Before sending, check to make sure data is realistic.  If not, don't send
//======================================================================================================================================
bool SendDataToXively()
{

  if ( g_gotNewData == true )
  {
    // Only send data out if it's in a valid range and sensor is working
    if(g_poolData[P_PRESSURE1] < 40.0 && isPreFltrPressSensorOk() )
    { datastreams[0].setFloat(g_poolData[P_PRESSURE1]); }  // 0 - Pre-filter pressure
    
    if(g_poolData[P_PRESSURE2] < 40.0 && isPostFltrPressSensorOk() )
    { datastreams[1].setFloat(g_poolData[P_PRESSURE2]); }  // 1- Post-Filter pressure
    
    if(g_poolData[P_PRESSURE3] < 100.0 && isWaterFillPressSensorOk() )
    { datastreams[2].setFloat(g_poolData[P_PRESSURE3]); }  // 2- water fill pressure
    
    if(g_poolData[P_TEMP1] > 40.0 && g_poolData[P_TEMP1] < 150.0 && isPreHtrTempSensorOk() )
    { datastreams[3].setFloat(g_poolData[P_TEMP1]); }    // 3 - Pre heater temp
    
    if(g_poolData[P_TEMP2] > 40.0 && g_poolData[P_TEMP2] < 150.0 && isPostHtrTempSensorOk() )
    { datastreams[4].setFloat(g_poolData[P_TEMP2]); }    // 4 - Post heater temp
    
    // Determine if Pool heater is on by comparing temperatures
    if( isPreHtrTempSensorOk() && isPostHtrTempSensorOk() && isPumpAmpsSensorOk() )
    {
      if(((g_poolData[P_TEMP2] - g_poolData[P_TEMP1]) > 5.0) && (g_poolData[P_PUMP_AMPS] > 5.0)  )
      { datastreams[5].setInt(1); }              // 5 - Heater status On/Off
      else
      { datastreams[5].setInt(0); }
    }
    
    if(g_poolData[P_TEMP_PUMP] > 40.0 && g_poolData[P_TEMP_PUMP] < 300.0 && isPumpTempSensorOk() )
    { datastreams[6].setFloat(g_poolData[P_TEMP_PUMP]); } // 6 - Pump temp
    
    if(g_poolData[P_PUMP_AMPS] < 50.0 && isPumpAmpsSensorOk() )
    { datastreams[7].setFloat(g_poolData[P_PUMP_AMPS]); } // 7 - Pump amps
    
    datastreams[8].setInt((int) g_poolData[P_LOW_PRES_CNT]); // 8 - Low pressure counter
    
    datastreams[9].setInt((int) g_poolData[P_WATER_FILL_MINUTES]); // 9 - Mintues water fill valve was on today
    
    datastreams[13].setInt((int) g_poolData[P_CONTROLLER_STATUS]); // 13 - Controller status code number
    
    datastreams[14].setInt(g_poolData[P_LOW_WATER] );              // 14 - water level sensor
    
    if (g_poolData[P_WATER_LVL_BATT] > 2000 )
    { datastreams[15].setFloat(g_poolData[P_WATER_LVL_BATT]/1000.0); } // 15 - battery volts for water level sensor
    
  } // end if(gotNewData)
  
  if( g_xBeeTimeoutFlag == true )
  { // No communication with XBEE
    #ifdef PRINT_DEBUG
      Serial.println(F("No Xbee Comm"));
    #endif
    datastreams[10].setBuffer("NO XBEE COMM");  // 10 - Controller status - sends text to Xively, not numbers
  }
  else
  {
    char txtStatus[STATUSBUFLEN];  // text buffer for controller status
    controllerStatus(txtStatus,  g_poolData[P_CONTROLLER_STATUS]);  // Convert controller status number to text
    datastreams[10].setBuffer(txtStatus);                           // 10 - Controller status - sends text to Xively, not numbers
  }
  
  datastreams[11].setInt(g_successes);                             // 11 - network successes
  datastreams[12].setInt( (millis() - g_xbeeLastRxTime) / 1000 );  // 12 - seconds since last xbee data received
  
  // Send data to Xively
  #ifdef PRINT_DEBUG
    Serial.println(F("\n\nSend to Xively"));
  #endif
  
  // push data to xively then print out status
  int ret = xivelyclient.put(feed, XIVELY_API_KEY);
  switch (ret)
  {
    case 200:
      g_xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT; // Reset upload timout timer
      g_successes++;
      g_failures = 0;
      #ifdef PRINT_DEBUG
        Serial.println(F("Successful Xively upload"));
      #endif

      return true;
      break;
    case HTTP_ERROR_CONNECTION_FAILED:
      g_failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nconnection to api.xively.com has failed. Failures = "));
        Serial.println(g_failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_API:
      g_failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nA method of HttpClient class was called incorrectly. Failures = "));
        Serial.println(g_failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_TIMED_OUT:
      g_failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nConnection with api.xively.com has timed-out. Failures = "));
        Serial.println(g_failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_INVALID_RESPONSE:
      g_failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nInvalid or unexpected response from the server. Failures = "));
        Serial.println(g_failures);
      #endif
      return false;
      break;
    default:
      g_failures++;
      #ifdef PRINT_DEBUG
        Serial.print(F("\nXively Unknown status: "));
        Serial.print(ret);
        Serial.print(F("  Failures = "));
        Serial.println(g_failures);
      #endif
      return false;
  }
}  // end SendDataToXively()



//=========================================================================================================
//==================================================================================================================
bool ReadXBeeData(uint16_t *Tx_Id)
{
  g_gotNewData = false; // reset flag
  
  // Read XBee data
  xbee.readPacket();
  
  if (xbee.getResponse().isAvailable())
  {
    // got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE )
    {
      // got a Rx packet
      xbee.getResponse().getRx16Response(rx16);  // I think this tells XBee to send the data over, not sure
      g_xbeeSignal = (int) rx16.getRssi();
//    #ifdef PRINT_DEBUG
//      Serial.print("RSSI ");
//      Serial.println(xbeeSignal);
//    #endif
      int dataLength = rx16.getDataLength();     // Get number of bytes of data sent from outside XBee
      uint16_t RxData[dataLength];               // Array to hold raw XBee data
      *Tx_Id = rx16.getRemoteAddress16();        // MY ID of Tx, remember MY is set as a hex number.  Useful if you have multiple transmitters
      
      // Convert data from bytes to integers
      for(int i=0; i < dataLength; i=i+2)
      {
        RxData[i/2]  = rx16.getData(i) << 8;
        RxData[i/2] |= rx16.getData(i+1);
        g_poolData[i/2] = (float) RxData[i/2] / 10.0; // value from XBee are 10x, convert back to normal size
      }
      
      // Put status bytes into byte variables
      g_sensorStatusByte = g_poolData[P_SENSORSTATUSBYTE];    // Each bit determines if sensor is operating properly
      g_ioStatusByte     = g_poolData[P_IOSTATUSBYTE];        // I/O state of digital I/O

      // If amps is low, it's really zero
      if ( g_poolData[P_PUMP_AMPS] < 0.25)
      { g_poolData[P_PUMP_AMPS] = 0.0; }
      
      g_gotNewData = true;
      g_xbeeLastRxTime = millis();  // Got data from xbee, so reset with current time
      g_xBeeTimeoutFlag = false;    // reset flag
      
      #ifdef PRINT_DEBUG
        PrintPoolData();
      #endif
      return true;
    }
    else // Didn't get a RX_16_RESPONSE
    {
      // Got a Response, but not RX_16_RESPONSE
      #ifdef PRINT_DEBUG
        Serial.println(F("Got XBee Response, but not RX_16_RESPONSE"));
      #endif
      return false;
    }
  }
  // XBee is not available
  else if (xbee.getResponse().isError())
  {
    // Got something, but not a packet
    // Can use xbee.getResponse().getErrorCode() to get an error number
    // You get error code 0 when the other Xbee isn't sending any data over
    return false;
  } // End Got Something
  else
  { return false; }  // xbee not available and no error
  
} // end ReadXbeeData()



//=========================================================================================================
// Log data to SD Card
//=========================================================================================================
bool logDataToSdCard(char * txtComment)
{

  // Build log file name: YYYYMMDD.log
  char logfile[13];
  sprintf(logfile, "%02d%02d%02d.log", year()-2000, month(), day());

  // Check to see if the file exists:
  if (!SD.exists(logfile))
  {
    // log file doesn't exist, create it
    File newFile = SD.open(logfile, FILE_WRITE);
    // Create header row
    newFile.print(F("Timestamp\tPool Time\tpre heat Temp\tpost heat temp\tpump temp\tamps\tpre fltr pressure\tpost fltr pressure\twtr fill pressure"));
    newFile.print(F("\tlow pressure cnt\twater fill min\tbattery\tlid\tlevel sensor (2 min)\tstatus ID\tstatus txt"));
     // sensorStatusbyte - sensor ok status
    newFile.print(F("\tPre-heat sensor ok\tpost-heat sensor ok\tpump temp sensor ok\tpre-filter sensor ok\tpost-filter sensor ok\tWater fill sensor ok\tamps sensor ok\tlevel sensor ok"));
    //  ioStatusbyte
    newFile.print(F("\tpump on/off relay\tauto switch\ton switch\tWater fill LED\tWater fill pb\tWater fill output\tHeater output\tlevel sensor realtime"));
    newFile.print(F("\tUpload Success\tfailures\txbee timeout\tRSSI\tXbee retries\tcomment"));
    newFile.println();
    newFile.close();
  }


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(logfile, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    char timebuf[24];
    sprintf(timebuf, " %02d/%02d/%d %02d:%02d:%02d", month(),day(),year(),hour(),minute(),second());
    dataFile.print(timebuf);
    dataFile.print(F("\t"));
    dataFile.print(g_poolData[P_POOL_TIME],2);
    dataFile.print(F("\t"));
    ( isPreHtrTempSensorOk() )     ? dataFile.print(g_poolData[P_TEMP1],1)     : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPostHtrTempSensorOk() )    ? dataFile.print(g_poolData[P_TEMP2],1)     : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPumpTempSensorOk() )       ? dataFile.print(g_poolData[P_TEMP_PUMP],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPumpAmpsSensorOk() )       ? dataFile.print(g_poolData[P_PUMP_AMPS],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPreFltrPressSensorOk() )   ? dataFile.print(g_poolData[P_PRESSURE1],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPostFltrPressSensorOk() )  ? dataFile.print(g_poolData[P_PRESSURE2],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isWaterFillPressSensorOk() ) ? dataFile.print(g_poolData[P_PRESSURE3],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    dataFile.print(g_poolData[P_LOW_PRES_CNT],0);
    dataFile.print(F("\t"));
    dataFile.print(g_poolData[P_WATER_FILL_MINUTES],0);
    dataFile.print(F("\t"));
    dataFile.print(g_poolData[P_WATER_LVL_BATT]/1000.0,2);  // battery volts
    dataFile.print(F("\t"));
    
    if ( (bool)g_poolData[P_LID_IS_LEVEL] )
    { dataFile.print(F("flat\t"));}
    else
    { dataFile.print(F("not flat\t"));}
    
    dataFile.print(g_poolData[P_LOW_WATER],0);  // Level sensor (2 minutes): 0 = level ok, 1 = low water, 2 = offline
    dataFile.print(F("\t"));
    
    
    dataFile.print(g_poolData[P_CONTROLLER_STATUS],0);    // controller status ID
    char txtStatus[STATUSBUFLEN];
    controllerStatus(txtStatus,  g_poolData[P_CONTROLLER_STATUS]);  // controller status text
    dataFile.print(F("\t"));
    dataFile.print(txtStatus);
    // export each bit in sensorStatusbyte
    for (byte i = 0; i < 8; i++)
    {
      dataFile.print(F("\t"));
      dataFile.print((g_sensorStatusByte >> i) & 1);
    }
    // export each bit in the ioStatusbyte
    for (byte i = 0; i < 8; i++)
    {
      dataFile.print(F("\t"));
      dataFile.print((g_ioStatusByte >> i) & 1);
    }
    dataFile.print(F("\t"));
    dataFile.print(g_successes);
    dataFile.print(F("\t"));
    dataFile.print(g_failures);
    dataFile.print(F("\t"));
    dataFile.print(g_xBeeTimeoutFlag);
    dataFile.print(F("\t"));
    dataFile.print(g_xbeeSignal);
    dataFile.print(F("\t"));
    dataFile.print(g_xbeeRetries);
    dataFile.print(F("\t"));
    dataFile.print(txtComment);
    dataFile.println();
    dataFile.close();

    Serial.print(F("Saved data to SD Card - "));
    Serial.println(logfile);
    return true;
  }
  // if the file isn't open, pop up an error:
  else
  {
    Serial.print(F("error opening "));
    Serial.println(logfile);
    return false;
  } 
  
}  // end logDataToSdCard()

//=========================================================================================================
// Print data sent from Pool Xbee
//=========================================================================================================
void PrintPoolData()
{
  static byte printheadcounter;
  
  // Print heading every 15 rows
  if (printheadcounter == 0)
  {
    Serial.println(F("\nTemp1\tTemp2\ttemp3\tAmps\tPres1\tPres2\tPres3\tP-Cnt\tWFmin\tbatt\tstat\tsesnsor\t\tI/O"));
    printheadcounter = 15;
  }
  
  printheadcounter--;
  
  Serial.print(g_poolData[P_TEMP1],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_TEMP2],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_TEMP_PUMP],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_PUMP_AMPS],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_PRESSURE1],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_PRESSURE2],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_PRESSURE3],1);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_LOW_PRES_CNT],0);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_WATER_FILL_MINUTES],0);
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_WATER_LVL_BATT]/1000.0,2);  // battery volts
  Serial.print(F("\t"));
  Serial.print(g_poolData[P_CONTROLLER_STATUS],0);
  Serial.print(F("\t"));
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(g_sensorStatusByte, j)); }  // print leading zeros
  Serial.print(F("\t"));
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(g_ioStatusByte, j)); }  // print leading zeros
  Serial.println();

/*  
  Serial.print(isPreFltrPressSensorOk()); 
  Serial.print(isPostFltrPressSensorOk());
  Serial.print(isWaterFillPressSensorOk());
  Serial.print(isPreHtrTempSensorOk());
  Serial.print(isPostHtrTempSensorOk());
  Serial.print(isPumpTempSensorOk());
  Serial.print(isPumpAmpsSensorOk());
  Serial.print(isWaterLevelSensorOk());
  Serial.println(); 
*/  
} // end PrintPoolData()


//=========================================================================================================
// Return the text for the pool controller status
// Longest text: 20 characters
//=========================================================================================================
void controllerStatus(char * txtStatus, int poolstatus)
{
  // txtStatus can hold 25 characters
  
  switch(poolstatus)
  {
    case 0:
      strcpy(txtStatus, "Pump Off");
      break;
    case 1:
      strcpy(txtStatus, "Pump On");
      break;
    case 2:
      strcpy(txtStatus, "Swtch Off");
      break;
    case 3:
      strcpy(txtStatus, "Water Filling");
      break;
    case 4:
      strcpy(txtStatus, "LO PRES Fluctuations");
      break;
    case 5:
      strcpy(txtStatus, "LO PRES 5 MIN");
      break;
    case 6:
      strcpy(txtStatus, "HI AMPS!");
      break;
    case 7:
      strcpy(txtStatus, "HI PUMP TEMP!");
      break;
    case 8:
      strcpy(txtStatus, "REMOTE SHUTDN");
      break;
    default:
      sprintf(txtStatus, "UNKNOWN STAT: %d", poolstatus);
      break;
  }
}  // end controllerStatus()



//=========================================================================================================
// Restarts program from beginning but does not reset the peripherals and registers
// Reference: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241733710
//=========================================================================================================
void software_Reset(void)
{
  asm volatile ("  jmp 0");
} // end software_Reset()


//=========================================================================================================
// Return the amount of free SRAM
// Parameters - true: Print out RAM, false: Don't print
// http://www.controllerprojects.com/2011/05/23/determining-sram-usage-on-arduino/
//=========================================================================================================
int freeRam(bool PrintRam)
{
  int freeSRAM;
  extern int __heap_start, *__brkval;
  int v;
  
  freeSRAM =  (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  
  if(PrintRam)
  {
    Serial.print(F("RAM "));
    Serial.println(freeSRAM);
  }
  return freeSRAM;

} // end freeRam()


// SRG - Move these to the class library

bool isPreFltrPressSensorOk()
{
  return ((g_sensorStatusByte >> 3) & 1);
}

bool isPostFltrPressSensorOk()
{
  return ((g_sensorStatusByte >> 4) & 1);
}

bool isWaterFillPressSensorOk()
{
  return ((g_sensorStatusByte >> 5) & 1);
}

bool isPreHtrTempSensorOk()
{
  return ((g_sensorStatusByte >> 0) & 1);
}

bool isPostHtrTempSensorOk()
{
  return ((g_sensorStatusByte >> 1) & 1);
}

bool isPumpTempSensorOk()
{
  return ((g_sensorStatusByte >> 2) & 1);
}

bool isPumpAmpsSensorOk()
{
  return ((g_sensorStatusByte >> 6) & 1);
}

bool isWaterLevelSensorOk()
{
  return ((g_sensorStatusByte >> 7) & 1);
}

bool isWaterFillValveOpen()
{
  return ((g_ioStatusByte >> 5) & 1);
}


