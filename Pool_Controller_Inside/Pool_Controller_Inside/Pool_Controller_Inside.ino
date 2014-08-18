/*
Xively Feed: http://xively.com/feeds/65673
Xbee Shield switch: to upload sketch, slide switch away from edge

Hardware: 
Arduino Mega
Xbee Series 1

To Do:
Delay after you receive an alarm and check again in a couple seconds
Try uploading to Xively without Xively.h library to see if you can reduce the program size
Tweet if heater is on and temp > 85

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
 
*/
#define VERSION "v1.59"
#define PRINT_DEBUG     // Comment out to turn off printing

#include <SPI.h>             // Communicate with SPI devices http://arduino.cc/en/Reference/SPI
#include <Ethernet.h>        // LIbrary for Arduino ethernet shield http://arduino.cc/en/Reference/Ethernet
#include <HttpClient.h>      // http://github.com/amcewen/HttpClient/blob/master/HttpClient.h
#include <Time.h>            // http://playground.arduino.cc/Code/time
#include <SPI.h>             // http://arduino.cc/en/Reference/SPI
#include <SD.h>              // Micro SD Card. http://arduino.cc/en/Reference/SD
#include "Xively.h"          // http://github.com/xively/xively_arduino
#include "Twitter.h"         // http://arduino.cc/playground/Code/TwitterLibrary, get token from token at http://arduino-tweet.appspot.com/
#include "XBee.h"            // http://code.google.com/p/xbee-arduino/
#include "Tokens.h"          // Tokens for Xively and twitter
#include "Pool_Controller_Inside_Library.h"    // Include application, user and local libraries

// This gets rid of compiler warning:  Only initialized variables can be placed into program memory area
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// I/O
const byte SD_CARD_SS =   4;
const byte SPI_ENABLE  = 53;  // Must be defined as an output for SPI to work properly
// D10 is reserved for Etherent SS


// Index positions for PoolData[] array
enum PoolDataIndex {
 P_TEMP1 = 0,           // Temperature before heater
 P_TEMP2,               // Temperature after
 P_TEMP_PUMP,           // Pump housing temperature
 P_PUMP_AMPS,           // Amps pump is using
 P_PRESSURE1,           // Pressure before filter
 P_PRESSURE2,           // Pressure after filter
 P_PRESSURE3,           // Pressure of water fill line
 P_LOW_PRES_CNT,        // Counts times pressure was low
 P_CONTROLLER_STATUS,   // Controller status 0-8
 P_WATER_FILL_MINUTES,  // Minutes water fill valve was open today
 P_LID_IS_LEVEL,        // Is lid level on water level sensor
 P_POOL_TIME,           // Pool time from RTC, 2:45 PM = 14.75
 P_WATER_LVL_BATT,      // Water level battery voltage
 P_LOW_WATER,           // Low water sensor: 0 = level ok, 1 = low water, 2 = offline
 P_SENSORSTATUSBYTE,    // Sensor Inputs Status Byte: 1 if sensor is working properly, 0 of not
 P_IOSTATUSBYTE,        // Discrete I/O status byte: shows on/off state if I/O
 NUM_POOL_DATA_PTS      // Number of data points in pool array xbee packet
};

byte sensorStatusByte;          // Each bit determines if sensor is operating properly
byte ioStatusByte;              // Each bit shows input value of digital I/O


// Number of Xively Streams
#define NUM_XIVELY_STREAMS  16

const byte STRLEN_MAX_TWEET =               75;   // Character array size for twitter message
const uint32_t XIVELY_UPDATE_INTERVAL =  15000;   // Xively upload interval (mS)
const uint32_t XIVELY_UPDATE_TIMEOUT = 1800000;   // 30 minute timeout - if there are no successful updates in 30 minutes, reboot
#define FEED_ID   65673                           // Xively Feed ID http://xively.com/feeds/65673
// #define FEED_ID 4663  // Test feed
uint32_t xively_uploadTimout_timer; // Timer to reboot if no successful uploads in 30 minutes
uint32_t xively_Upload_Timer;       // Timer for uploading to Xively

const int bufferSize = 30;
char bufferValue[bufferSize]; // enough space to store the string we're going to send to Xively

const byte statusBufLen = 26;  // character buffer length for controller status text

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
  XivelyDatastream("10", 2, DATASTREAM_BUFFER, bufferValue, bufferSize), // Status of controller.  Only stream where text is sent instead of a number
  XivelyDatastream("11", 2, DATASTREAM_INT),    // Xively upload successes
  XivelyDatastream("12", 2, DATASTREAM_INT),    // Xively Network Failures
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
uint8_t successes = 0;    // Xively upload success, will rollover at 255, but that's okay.  This makes is easy to see on Xively is things are running
uint8_t failures =  0;    // Xively upload failures


// NPT Time setup
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
const int timeZone = -4;  // Eastern Standard Time (USA)
EthernetUDP Udp;
const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming & outgoing packets


// Xbee Setup 
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();

bool gotNewData= false;        // Flag to indicate that sketch has received new data from xbee
uint32_t xbeeLastRxTime;       // Last time data was received from xbee
bool xBeeTimeoutFlag = false;  // Flag to indicate no data from Xbee, used to keep warning from going off every 5 minutes
int8_t xbeeSignal;             // xbee RSSI Signal strength
uint8_t xbeeRetries; 
const int chipSelect = 4;  // Micro SD Card

// Twitter setup
Twitter twitter(TWITTER_TOKEN);


// Array to hold pool data received from outside controller
float PoolData[NUM_POOL_DATA_PTS];  


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
void sendNTPpacket(IPAddress &address);
time_t getNtpTime();

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
  delay(2000);
  
  #ifdef PRINT_DEBUG
    Serial.print(F("\nSetup pool controller inside "));
    Serial.println(VERSION);
  #endif

  // Xbee is using Serial1 on Mega pins D18 (Tx) D19 (Rx)
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  // Initialize Ethernet
  pinMode(SPI_ENABLE, OUTPUT);
  pinMode(SD_CARD_SS, OUTPUT);
  digitalWrite(SD_CARD_SS, HIGH);  // disable microSD card interface while ethernet is starting
  Ethernet.begin(mac);
  delay(1000);

  unsigned int localPort = 8888;    // local port to listen for UDP packets
  Serial.println(Ethernet.localIP());
  Udp.begin(localPort);

  // Setup NTP time
  setSyncProvider(getNtpTime);  // tells time library to call getNtpTime() when it wants to sync arduino with NTP time
  // Get time from UDP server
  if ( getNtpTime() == 0 )
  {
    // Couldn't get time from UDP Server, try one more tiem
    delay(500);
    getNtpTime();
  }
  //  setTime(hour(t),minute(t),second(t),month(t),day(t),year(t));
  #ifdef PRINT_DEBUG
    char timebuf[16];
    sprintf(timebuf, "Time: %02d:%02d:%02d", hour(),minute(),second());
    Serial.println(timebuf);
  #endif
  	
  // Initialize SD Card
  if ( SD.begin(chipSelect) )
  #ifdef PRINT_DEBUG
    { Serial.println(F("card initialized")); }
    else
    { Serial.println(F("Card failed, or not present")); }
  #endif
  
  // Initialize global variables
  xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
  xively_Upload_Timer       = millis() + XIVELY_UPDATE_INTERVAL;
    
  xbeeLastRxTime = millis();  // initialize timer, didn't get xbee data yet, but don't want to generate an error on startup
  
  #ifdef PRINT_DEBUG
    Serial.println(F("End setup"));
    freeRam(true);
  #endif
  
} // setup()


//=========================================================================================================
//============================================================================
void loop(void)
{
  uint16_t xbeeID;                      // ID of transimitting xbee
  char msgTweet[STRLEN_MAX_TWEET];      // Holds text for twitter message.  Should be big enough for message and timestamp
  static bool gotFistXbeeData = false;  // Needed at reboot to prevent false alarms for sensor status.  Set to true after first good xbee transmit 
  
  // Read XBee data
  // Keep tying to read data until successful.  After 30 tries, give up and move on
  bool xbeeStat;
  byte xbeeFailCnt = 0;
  do
  {
    delay(250);
    xbeeStat = ReadXBeeData(&xbeeID);
    xbeeFailCnt++;
  } while (xbeeStat == false && xbeeFailCnt < 30 );
  xbeeRetries = xbeeFailCnt;
  
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
  if( (long)(millis() - xbeeLastRxTime) >= 30000L )
  { xBeeTimeoutFlag = true; }

    // Log water level sensor status
//srg    if ( xBeeTimeoutFlag == false && PoolData[P_LOW_WATER] == 1)
//    { logDataToSdCard(""); }

  // Upload data to Xively
  if ((long)(millis() - xively_Upload_Timer) >= 0)
  {
    xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL;  // Reset timer
    SendDataToXively(); // Send Data to Xively
    
    // Log data to SD card, if we are receiving data from outside controller
    char emptyString[] = "";
    if ( xBeeTimeoutFlag == false )
    { logDataToSdCard(emptyString); }
  }
  
  // Check inputs and send message (Tweet) if anything is wrong
  // Only do this if we have establisehd communication with Xbee at least once
  if ( gotFistXbeeData )
  { checkAlarms(); }
  
  // Reboot if no successful updates to Xively in 30 minutes
  if((long) (millis() - xively_uploadTimout_timer) >= 0)
  {
    strcpy(msgTweet, "Reboot: can't upload to Xively.");
    SendTweet(msgTweet);
    xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
    delay(1000);
    software_Reset();
  }
  
} // loop()


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
  static alarmFlag tf_Xbee_Comm;          // Xbee communication, send alert if no comm
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
  if( PoolData[P_PUMP_AMPS] >= 17.0 && tf_highAmps != ALARM_2 )
  {
    if( tf_highAmps == NO_ALARM )
    { 
      tf_highAmps = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    {
      sprintf(msgAlarm, "High pump amps: %d.", (int) PoolData[P_PUMP_AMPS]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highAmps = ALARM_2;
    }
  }

  // Reset high amps flag
  if( PoolData[P_PUMP_AMPS] < 10 )
  { tf_highAmps = NO_ALARM; }

  // High pump temperature
  if( PoolData[P_TEMP_PUMP] >= 175.0 && tf_highPumpTemp != ALARM_2 )
  {
    if (tf_highPumpTemp == NO_ALARM )
    {
      tf_highPumpTemp = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      sprintf(msgAlarm, "High pump temp: %d.", (int) PoolData[P_TEMP_PUMP]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highPumpTemp =  ALARM_2; 
    }
  }

  // Reset high pump temp flag
  if( PoolData[P_TEMP_PUMP] < 110 )
  { tf_highPumpTemp = NO_ALARM; }
  
  // High pressure
  if( PoolData[P_PRESSURE1] >= 40.0 && tf_highPressure != ALARM_2 )
  {
    if ( tf_highPressure == NO_ALARM )
    {
      tf_highPressure = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      sprintf(msgAlarm, "High pump pressure: %d.", (int) PoolData[P_PRESSURE1]);
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_highPressure = ALARM_2;
    }
  }

  // Reset high pressure flag
  if( PoolData[P_PRESSURE1] < 110 )
  { tf_highPressure = NO_ALARM; }

  // Check pressure drop across filter
  if( (PoolData[P_PRESSURE1] > 25.0) && ((PoolData[P_PRESSURE1] - PoolData[P_PRESSURE2]) > 10.0) && (tf_highPresDrop != ALARM_2) )
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
  if( PoolData[P_PRESSURE1] < 2 )
  { tf_highPresDrop = NO_ALARM; }
  
  // Check low pressure Counter
  if( PoolData[P_LOW_PRES_CNT] >= 17.0 && tf_lowPressure != ALARM_2  )
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
  if( PoolData[P_LOW_PRES_CNT] == 0 )
  { tf_lowPressure = NO_ALARM; }

  // Check for emergency shutdown and send tweet
  if( PoolData[P_CONTROLLER_STATUS] >= 4 && tf_emergencyShutdown != ALARM_2 )
  {
    if( tf_emergencyShutdown == NO_ALARM )
    {
      tf_emergencyShutdown = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      char txtStatus[statusBufLen];
      strcpy(msgAlarm, "Emergency Shutdown - ");
      controllerStatus(txtStatus,  PoolData[P_CONTROLLER_STATUS]);
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
  if( PoolData[P_CONTROLLER_STATUS] < 4 )
  { tf_emergencyShutdown = NO_ALARM; }
  
  // Send Tweet if water fill has started
  if( isWaterFillValveOpen() && tf_waterFillOn != ALARM_2 )
  {
    // Wait a couple seconds in case water fill button is pressed a couple times, then read XBee data again
    delay(2500);
    uint16_t xbeeID;
    ReadXBeeData(&xbeeID);
    strcpy(msgAlarm, "Water fill started");
    logDataToSdCard(msgAlarm);
    SendTweet(msgAlarm);
    tf_waterFillOn = ALARM_2;  // flag so Tweet is only sent once
  }

  // Reset water fill timer flag
  if( isWaterFillValveOpen() == false )
  { tf_waterFillOn = NO_ALARM; }

  // Send Tweet if pump is running at night
  if( PoolData[P_PUMP_AMPS] > 6.0 && PoolData[P_POOL_TIME] >= 20.3 && tf_pumpOnAtNight != ALARM_2  )  // 8:30 PM
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
  if( PoolData[P_PUMP_AMPS] < 2.0 && PoolData[P_POOL_TIME] > 8.0 && PoolData[P_POOL_TIME] < 17.0 && tf_pumpOffInDay != ALARM_2  )
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
  if( ((PoolData[P_TEMP2] - PoolData[P_TEMP1]) > 5.0) && (PoolData[P_PUMP_AMPS] > 5.0) && tf_heaterIsOn != ALARM_2 )
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

  // Send alert if xbee communication is lost for 10 mintues
  if( (long)(millis() - xbeeLastRxTime) > 600000L  && tf_Xbee_Comm != ALARM_2 )
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
    }
  }
  
  // Reset xbee communication flag
  if( gotNewData == true )
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
  if ( !isWaterLevelSensorOk() && tf_waterLevelSensor != ALARM_2 )
  {
    if( tf_waterLevelSensor == NO_ALARM )
    {
      tf_waterLevelSensor = ALARM_1; 
      delay(ALARM_DELAY);
    }
    else
    { 
      strcpy(msgAlarm, "Pool water level sensor trouble");
      logDataToSdCard(msgAlarm); 
      SendTweet(msgAlarm);
      tf_waterLevelSensor = ALARM_2;
    }
  }
  
  // Reset water Level sensor flag
  if ( isWaterLevelSensorOk() )
  { tf_waterLevelSensor = NO_ALARM;}

  // At 11:01 PM reset flags for pump running at night, not running in day, and heater is on
  if(hour() == 23 && minute() == 1 && second() < 10 )
  {
    // Reset twitter flags
    tf_pumpOnAtNight = NO_ALARM;
    tf_pumpOffInDay =  NO_ALARM;
    tf_heaterIsOn =    NO_ALARM;
    delay(10000); // delay 10 seconds to prevent multiple tweets from going out
  }

  // tf_pumpOffInDay flag can also be reset if pump it turned back on
  if(PoolData[P_PUMP_AMPS] > 6 && ((sensorStatusByte >> 6) & 1) == 1)
  { tf_pumpOffInDay = NO_ALARM; }

} // end checkAlarms()


//=========================================================================================================
// Send twitter text, appends the time to the message to avoid twitter blocking duplicate messages
// bitly link to xively: bit.ly/1vapvvo
//=========================================================================================================
int SendTweet(char * txtTweet)
{
  const uint8_t STRLEN_TIMESTAMP = 20; 
  const uint8_t STRLEN_BITLY = 15;
  
  xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL; // increase timer for Xively so it doesn't send right after Twitter

  logDataToSdCard(txtTweet); // update log file

  char cpoolTime[STRLEN_TIMESTAMP + STRLEN_BITLY];   // char arry to hold pool time & bitly URL
  
  if(strlen(txtTweet) <= STRLEN_MAX_TWEET - STRLEN_TIMESTAMP - STRLEN_BITLY) // Make sure message there is room in character array for the timestamp & bitly
  {
//    sprintf(cpoolTime, " Pool Time: %01d:%02d", int(floor(fpoolTime)), (int)((fpoolTime - floor(fpoolTime)) * 60));
    sprintf(cpoolTime, " Time: %d:%02d\n bit.ly/1vapvvo", hour(), minute());
    strcat(txtTweet, cpoolTime);          // Append time to the message
  }
  
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

  if ( gotNewData == true )
  {
    // Only send data out if it's in a valid range and sensor is working
    if(PoolData[P_PRESSURE1] < 40.0 && isPreFltrPressSensorOk() )
    { datastreams[0].setFloat(PoolData[P_PRESSURE1]); }  // 0 - Pre-filter pressure
    
    if(PoolData[P_PRESSURE2] < 40.0 && isPostFltrPressSensorOk() )
    { datastreams[1].setFloat(PoolData[P_PRESSURE2]); }  // 1- Post-Filter pressure
    
    if(PoolData[P_PRESSURE3] < 100.0 && isWaterFillPressSensorOk() )
    { datastreams[2].setFloat(PoolData[P_PRESSURE3]); }  // 2- water fill pressure
    
    if(PoolData[P_TEMP1] > 40.0 && PoolData[P_TEMP1] < 150.0 && isPreHtrTempSensorOk() )
    { datastreams[3].setFloat(PoolData[P_TEMP1]); }    // 3 - Pre heater temp
    
    if(PoolData[P_TEMP2] > 40.0 && PoolData[P_TEMP2] < 150.0 && isPostHtrTempSensorOk() )
    { datastreams[4].setFloat(PoolData[P_TEMP2]); }    // 4 - Post heater temp
    
    // Determine if Pool heater is on by comparing temperatures
    if( isPreHtrTempSensorOk() && isPostHtrTempSensorOk() && isPumpAmpsSensorOk() )
    {
      if(((PoolData[P_TEMP2] - PoolData[P_TEMP1]) > 5.0) && (PoolData[P_PUMP_AMPS] > 5.0)  )
      { datastreams[5].setInt(1); }              // 5 - Heater status On/Off
      else
      { datastreams[5].setInt(0); }
    }
    
    if(PoolData[P_TEMP_PUMP] > 40.0 && PoolData[P_TEMP_PUMP] < 300.0 && isPumpTempSensorOk() )
    { datastreams[6].setFloat(PoolData[P_TEMP_PUMP]); } // 6 - Pump temp
    
    if(PoolData[P_PUMP_AMPS] < 50.0 && isPumpAmpsSensorOk() )
    { datastreams[7].setFloat(PoolData[P_PUMP_AMPS]); } // 7 - Pump amps
    
    datastreams[8].setInt((int) PoolData[P_LOW_PRES_CNT]); // 8 - Low pressure counter
    
    datastreams[9].setInt((int) PoolData[P_WATER_FILL_MINUTES]); // 9 - Mintues water fill valve was on today
    
    datastreams[13].setInt((int) PoolData[P_CONTROLLER_STATUS]); // 13 - Controller status code number
    
    datastreams[14].setInt(PoolData[P_LOW_WATER] ); // 14 - water level sensor
    datastreams[15].setFloat(PoolData[P_WATER_LVL_BATT]/1000.0); // 15 - battery volts for water level sensor
    
  } // end if(gotNewData)
  
  if( xBeeTimeoutFlag == true )
  { // No communication with XBEE
    #ifdef PRINT_DEBUG
      Serial.println(F("No Xbee Comm"));
    #endif
    datastreams[10].setBuffer("NO XBEE COMM");  // 10 - Controller status - sends text to Xively, not numbers
  }
  else
  {
    char txtStatus[statusBufLen];  // text buffer for controller status
    controllerStatus(txtStatus,  PoolData[P_CONTROLLER_STATUS]);  // Convert controller status number to text
    datastreams[10].setBuffer(txtStatus);                           // 10 - Controller status - sends text to Xively, not numbers
  }
  
  datastreams[11].setInt(successes); // 11 - network successes
  datastreams[12].setInt(failures);  // 12 - network failures
  
  // Send data to Xively
  #ifdef PRINT_DEBUG
    Serial.println(F("\n\nSend to Xively"));
  #endif
  
  int ret = xivelyclient.put(feed, XIVELY_API_KEY);
  switch (ret)
  {
    case 200:
      xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT; // Reset upload timout timer
      successes++;
      failures = 0;
      #ifdef PRINT_DEBUG
        Serial.println(F("Successful Xively upload"));
      #endif
      // Flash twice when we send data to Xively successfully

      return true;
      break;
    case HTTP_ERROR_CONNECTION_FAILED:
      failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nconnection to api.xively.com has failed. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_API:
      failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nA method of HttpClient class was called incorrectly. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_TIMED_OUT:
      failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nConnection with api.xively.com has timed-out. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_INVALID_RESPONSE:
      failures++;
      #ifdef PRINT_DEBUG
        Serial.println(F("\nInvalid or unexpected response from the server. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    default:
      failures++;
      #ifdef PRINT_DEBUG
        Serial.print(F("\nXively Unknown status: "));
        Serial.print(ret);
        Serial.print(F("  Failures = "));
        Serial.println(failures);
      #endif
      return false;
  }
}  // end SendDataToXively()



//=========================================================================================================
//==================================================================================================================
bool ReadXBeeData(uint16_t *Tx_Id)
{
  gotNewData = false; // reset flag
  
  // Read XBee data
  xbee.readPacket();
  
  if (xbee.getResponse().isAvailable())
  {
    // got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE )
    {
      // got a Rx packet
      xbee.getResponse().getRx16Response(rx16);  // I think this tells XBee to send the data over, not sure
      xbeeSignal = (int) rx16.getRssi();
//    #ifdef PRINT_DEBUG
//      Serial.print("RSSI ");
//      Serial.println(xbeeSignal);
//    #endif
      int dataLength = rx16.getDataLength();     // Get number of bytes of data sent from outside XBee
      uint16_t RxData[dataLength];               // Array to hold raw XBee data
      *Tx_Id = rx16.getRemoteAddress16();        // MY ID of Tx, remember MY is set as a hex number.  Useful if you have multiple transmitters
      
      // Convert data from bytes to integers
      for(int i=0; i < dataLength - 2; i=i+2)
      {
        RxData[i/2]  = rx16.getData(i) << 8;
        RxData[i/2] |= rx16.getData(i+1);
        PoolData[i/2] = (float) RxData[i/2] / 10.0; // value from XBee are 10x, convert back to normal size
      }
      
      // Put status bytes into byte variables
      sensorStatusByte = PoolData[P_SENSORSTATUSBYTE];    // Each bit determines if sensor is operating properly
      ioStatusByte     = PoolData[P_IOSTATUSBYTE];        // I/O state of digital I/O
      
      gotNewData = true;
      xbeeLastRxTime = millis();  // Got data from xbee, so reset with current time
      xBeeTimeoutFlag = false;    // reset flag
      
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
  
} // ReadXbeeData()



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
    dataFile.print(PoolData[P_POOL_TIME],2);
    dataFile.print(F("\t"));
    ( isPreHtrTempSensorOk() )     ? dataFile.print(PoolData[P_TEMP1],1)     : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPostHtrTempSensorOk() )    ? dataFile.print(PoolData[P_TEMP2],1)     : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPumpTempSensorOk() )       ? dataFile.print(PoolData[P_TEMP_PUMP],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPumpAmpsSensorOk() )       ? dataFile.print(PoolData[P_PUMP_AMPS],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPreFltrPressSensorOk() )   ? dataFile.print(PoolData[P_PRESSURE1],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isPostFltrPressSensorOk() )  ? dataFile.print(PoolData[P_PRESSURE2],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    ( isWaterFillPressSensorOk() ) ? dataFile.print(PoolData[P_PRESSURE3],1) : dataFile.print(F("n/a"));
    dataFile.print(F("\t"));
    dataFile.print(PoolData[P_LOW_PRES_CNT],0);
    dataFile.print(F("\t"));
    dataFile.print(PoolData[P_WATER_FILL_MINUTES],0);
    dataFile.print(F("\t"));
    dataFile.print(PoolData[P_WATER_LVL_BATT]/1000.0,2);  // battery volts
    dataFile.print(F("\t"));
    
    if ( (bool)PoolData[P_LID_IS_LEVEL] )
    { dataFile.print(F("flat\t"));}
    else
    { dataFile.print(F("not flat\t"));}
    
    dataFile.print(PoolData[P_LOW_WATER],0);  // Level sensor (2 minutes): 0 = level ok, 1 = low water, 2 = offline
    dataFile.print(F("\t"));
    
    
    dataFile.print(PoolData[P_CONTROLLER_STATUS],0);    // controller status ID
    char txtStatus[statusBufLen];
    controllerStatus(txtStatus,  PoolData[P_CONTROLLER_STATUS]);  // controller status text
    dataFile.print(F("\t"));
    dataFile.print(txtStatus);
    // export each bit in sensorStatusbyte
    for (byte i = 0; i < 8; i++)
    {
      dataFile.print(F("\t"));
      dataFile.print((sensorStatusByte >> i) & 1);
    }
    // export each bit in the ioStatusbyte
    for (byte i = 0; i < 8; i++)
    {
      dataFile.print(F("\t"));
      dataFile.print((ioStatusByte >> i) & 1);
    }
    dataFile.print(F("\t"));
    dataFile.print(successes);
    dataFile.print(F("\t"));
    dataFile.print(failures);
    dataFile.print(F("\t"));
    dataFile.print(xBeeTimeoutFlag);
    dataFile.print(F("\t"));
    dataFile.print(xbeeSignal);
    dataFile.print(F("\t"));
    dataFile.print(xbeeRetries);
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
  
}  // logDataToSdCard

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
  
  Serial.print(PoolData[P_TEMP1],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_TEMP2],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_TEMP_PUMP],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_PUMP_AMPS],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_PRESSURE1],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_PRESSURE2],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_PRESSURE3],1);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_LOW_PRES_CNT],0);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_WATER_FILL_MINUTES],0);
  Serial.print(F("\t"));
  Serial.print(PoolData[P_WATER_LVL_BATT]/1000.0,2);  // battery volts
  Serial.print(F("\t"));
  Serial.print(PoolData[P_CONTROLLER_STATUS],0);
  Serial.print(F("\t"));
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(sensorStatusByte, j)); }  // print leading zeros
  Serial.print(F("\t"));
  for (int j = 7; j >= 0; j--)
  { Serial.print(bitRead(ioStatusByte, j)); }  // print leading zeros
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
} // PrintPoolData()


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
}  //controllerStatus()



//=========================================================================================================
// Restarts program from beginning but does not reset the peripherals and registers
// Reference: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1241733710
//=========================================================================================================
void software_Reset(void)
{
  asm volatile ("  jmp 0");
} // End software_Reset()


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

} // freeRam()


// -------- NTP code ----------
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}  // getNtpTime()

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}  // sendNTPpacket()


// SRG - Move these to the class library when you build that out
// Check pre filter pressure sensor
bool isPreFltrPressSensorOk()
{
  return ((sensorStatusByte >> 3) & 1);
}

bool isPostFltrPressSensorOk()
{
  return ((sensorStatusByte >> 4) & 1);
}

bool isWaterFillPressSensorOk()
{
  return ((sensorStatusByte >> 5) & 1);
}

bool isPreHtrTempSensorOk()
{
  return ((sensorStatusByte >> 0) & 1);
}

bool isPostHtrTempSensorOk()
{
  return ((sensorStatusByte >> 1) & 1);
}

bool isPumpTempSensorOk()
{
  return ((sensorStatusByte >> 2) & 1);
}

bool isPumpAmpsSensorOk()
{
  return ((sensorStatusByte >> 6) & 1);
}

bool isWaterLevelSensorOk()
{
  return ((sensorStatusByte >> 7) & 1);
}

bool isWaterFillValveOpen()
{
  return ((ioStatusByte >> 5) & 1);
}


