/*
 Xively Feed: http://xively.com/feeds/65673/workbench
 
 Xbee Shield switch: to upload sketch, slide switch away from edge
 
 */

#define PRINT_DEBUG     // Comment out to turn off serial printing

#include <SPI.h>             // Communicate with SPI devices http://arduino.cc/en/Reference/SPI
#include <Ethernet.h>        // LIbrary for Arduino ethernet shield http://arduino.cc/en/Reference/Ethernet
#include <HttpClient.h>      // https://github.com/amcewen/HttpClient/blob/master/HttpClient.h
#include <Xively.h>          // http://github.com/xively/xively_arduino
#include <Twitter.h>         // http://arduino.cc/playground/Code/TwitterLibrary, get token from token at http://arduino-tweet.appspot.com/
#include <XBee.h>            // http://code.google.com/p/xbee-arduino/     Modified per http://arduino.cc/forum/index.php/topic,111354.0.html
#include <Tokens.h>          // Tokens for Xively and twitter
#include "LocalLibrary.h"    // Include application, user and local libraries

#define BAUD_RATE 9600  // Baud for bith Xbee and serial monitor

// Array positions for pool data array PoolData[]
#define P_TEMP1               0   // Temperature before heater
#define P_TEMP2               1   // Temperature after
#define P_TEMP_PUMP           2   // Pump housing temperature
#define P_PUMP_AMPS           3   // Amps pump is using
#define P_PRESSURE1           4   // Pressure before filter
#define P_PRESSURE2           5   // Pressure after filter
#define P_PRESSURE3           6   // Pressure of water fill line
#define P_LOW_PRES_CNT        7   // Counts times pressure was low
#define P_CONTROLLER_STATUS   8   // Controller status 0-8
#define P_WATER_FILL_MINUTES  9   // Minutes water fill valve was open today
#define P_WATER_FILL_COUNTDN 10   // Countdown timer for water fill valve
#define P_POOL_TIME          11   // Pool time from RTC, 2:45 PM = 14.75
#define P_WATER_LVL_BATT     12   // Water level battery voltage
#define P_LOW_WATER          13   // Low water sensor: 0 = level ok, 1 = low water, 2 = offline
#define P_SENSORSTATUSBYTE   14   // Sensor Inputs Status Byte: 1 if sensor is working properly, 0 of not
#define P_IOSTATUSBYTE       15   // Discrete I/O status byte: shows on/off state if I/O
#define NUM_POOL_DATA_PTS    16   // Number of data points in pool array xbee packet

byte sensorStatusbyte;          // Each bit determines if sensor is operating properly
byte ioStatusbyte;              // Each bit shows input value of digital I/O

/*
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
 */

// Xively Stream IDs
#define STREAM_PRESSURE1           "0"   // Pressure before filter
#define STREAM_PRESSURE2           "1"   // Pressure after filter
#define STREAM_PRESSURE3           "2"   // Water Fill Pressure
#define STREAM_TEMP1               "3"   // Temp before heater
#define STREAM_TEMP2               "4"   // Temp after heater
#define STREAM_HTR_STATUS          "5"   // Heater On/Off Status
#define STREAM_TEMP_PUMP           "6"   // Pump housing temperature
#define STREAM_PUMP_AMPS           "7"   // Pump amps
#define STREAM_LOW_PRES_CNT        "8"   // Low Pressure Count
#define STREAM_FILL_MINUTES        "9"   // Minutes water fill valve was otpen today
#define STREAM_CTRL_STATUS_TXT    "10"   // Status of controller - text
#define STREAM_SUCCESS            "11"   // Xively upload successes
#define STREAM_FAILURE            "12"   // Xively Network Failures
#define STREAM_CTRL_STATUS_CODE   "13"   // Status of controller - number
#define STREAM_LEVEL_SENSOR       "14"   // Water level sensor
#define STREAM_BATTERY            "15"   // battery volts for water level sensor
#define NUM_XIVELY_STREAMS         16

#define TWEETMAXSIZE              60   // Character array size for twitter message
#define XIVELY_UPDATE_INTERVAL   15000   // Xively upload interval (mS)
#define XIVELY_UPDATE_TIMEOUT  1800000   // 30 minute timeout - if there are no successful updates in 30 minutes, reboot
#define FEED_ID 65673                  // Xively Feed ID http://xively.com/feeds/65673/workbench
// #define FEED_ID 4663  // Test feed
uint32_t xively_uploadTimout_timer; // Timer to reboot if no successful uploads in 30 minutes
uint32_t xively_Upload_Timer;       // Timer for uploading to Xively

const int bufferSize = 30;
char bufferValue[bufferSize]; // enough space to store the string we're going to send

const byte statusBufLen = 26;  // character buffer length for controller status text

XivelyDatastream datastreams[] =
{
  XivelyDatastream(STREAM_PRESSURE1,        strlen(STREAM_PRESSURE1),        DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_PRESSURE2,        strlen(STREAM_PRESSURE2),        DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_PRESSURE3,        strlen(STREAM_PRESSURE3),        DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_TEMP1,            strlen(STREAM_TEMP1),            DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_TEMP2,            strlen(STREAM_TEMP2),            DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_HTR_STATUS,       strlen(STREAM_HTR_STATUS),       DATASTREAM_INT),
  XivelyDatastream(STREAM_TEMP_PUMP,        strlen(STREAM_TEMP_PUMP),        DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_PUMP_AMPS,        strlen(STREAM_PUMP_AMPS),        DATASTREAM_FLOAT),
  XivelyDatastream(STREAM_LOW_PRES_CNT,     strlen(STREAM_LOW_PRES_CNT),     DATASTREAM_INT),
  XivelyDatastream(STREAM_FILL_MINUTES,     strlen(STREAM_FILL_MINUTES),     DATASTREAM_INT),
  XivelyDatastream(STREAM_CTRL_STATUS_TXT,  strlen(STREAM_CTRL_STATUS_TXT),  DATASTREAM_BUFFER, bufferValue, bufferSize),
  XivelyDatastream(STREAM_SUCCESS,          strlen(STREAM_SUCCESS),          DATASTREAM_INT),
  XivelyDatastream(STREAM_FAILURE,          strlen(STREAM_FAILURE),          DATASTREAM_INT),
  XivelyDatastream(STREAM_CTRL_STATUS_CODE, strlen(STREAM_CTRL_STATUS_CODE), DATASTREAM_INT),
  XivelyDatastream(STREAM_LEVEL_SENSOR,     strlen(STREAM_LEVEL_SENSOR),     DATASTREAM_INT),
  XivelyDatastream(STREAM_BATTERY,          strlen(STREAM_BATTERY),          DATASTREAM_FLOAT)
};

// Wrap the datastreams into a feed
XivelyFeed feed(FEED_ID, datastreams, NUM_XIVELY_STREAMS);


// Ethernet Setup
EthernetClient client;
XivelyClient xivelyclient(client);
byte mac[] = { 0xCC, 0xAC, 0xBE, 0x21, 0x91, 0x43 };
uint8_t successes = 0;    // Xively upload success, will rollover at 255, but that's okay.  This makes is easy to see on Xively is things are running
uint8_t failures =  0;    // Xively upload failures


// Xbee Setup stuff
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
uint8_t xbeeErrors;      // XBee Rx errors
bool gotNewData;         // Flag to indicate that sketch has received new data from xbee
uint32_t xbeeTimeout;    // Counts time between successful Xbee data, if it goes too long, it means we've lost our connection to Xbee
bool xBeeTimeoutFlag;    // Flag to indicate no date from Xbee, used to keep warning from going off every 5 minutes


// Twitter setup
Twitter twitter(TWITTER_TOKEN);


// I/O Setup
#define LED_XBEE_ERROR       6  // LED flashes when XBee error
#define LED_XBEE_SUCCESS     7  // LED flashes when XBee success
#define LED_XIVELY_ERROR     6  // LED flashes when Xively error
#define LED_XIVELY_SUCCESS   5  // LED flashes when Xively success


// Array to hold pool data received from outside controller
float PoolData[NUM_POOL_DATA_PTS];  


// Declare function prototypes
void PrintPoolData();
void flashLed(int pin, int times, int wait);
void software_Reset();
bool SendDataToXively();
bool ReadXBeeData(uint16_t *Tx_Id);
void sendAlarmMessage();
int SendTweet(char msgTweet[], double fpoolTime);
int freeRam(bool PrintRam);
void controllerStatus(char * txtStatus, int poolstatus);





//=========================================================================================================
//============================================================================
void setup(void)
{
  delay(1000);
  pinMode(LED_XBEE_ERROR, OUTPUT);
  pinMode(LED_XBEE_SUCCESS, OUTPUT);
  pinMode(LED_XIVELY_ERROR, OUTPUT);
  pinMode(LED_XIVELY_SUCCESS, OUTPUT);
  
  
  // disable microSD card interface on Ethernet shield
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  Serial.begin(BAUD_RATE);
  
  #ifdef PRINT_DEBUG
    Serial.println(F("\nSetup pool controller inside"));
  #endif
  
  // Initialize XBee
  xbee.begin(BAUD_RATE);
  
  // Initialize Ethernet
  Ethernet.begin(mac);
  delay(1000);
  
  
  // Setup flashes LEDs so you know Arduino is booting up
  // parameters, Pin#, times to flash, wait time
  flashLed(LED_XBEE_ERROR,   6, 40);
  flashLed(LED_XBEE_SUCCESS, 6, 40);
  flashLed(LED_XIVELY_ERROR,   6, 40);
  flashLed(LED_XIVELY_SUCCESS, 6, 40);
  
  // Initialize global variables
  xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
  xively_Upload_Timer       = millis() + XIVELY_UPDATE_INTERVAL;
  
  
  xbeeTimeout       = millis() + 300000UL; // 5 minutes
  xBeeTimeoutFlag   = false;
  gotNewData        = false;  // Initialize, true when new we receive new data from xbee, is reset when it uploads to Xively
  
#ifdef PRINT_DEBUG
  Serial.println(F("End setup"));
  freeRam(true);
#endif
}  //setup()


//=========================================================================================================
//============================================================================
void loop(void)
{
  
  
  uint16_t xbeeID;                // ID of transimitting xbee
  char msgTweet[TWEETMAXSIZE];    // Holds text for twitter message.  Should be big enough for message and timestamp
  
  
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
  
  // Send a Tweet for startup, do this after you read xbee data so you can append pooltime which avoids duplicate tweets
  static bool tweetstartup;
  if (tweetstartup == false && millis() > 20000)
  {
    strcpy(msgTweet, "Inside pool monitor has restarted.");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
    tweetstartup = true;
  }
  
  // Check for Xbee Timeout: 5 minutes
  if(((long)(millis() - xbeeTimeout) >= 0 ) && ( xBeeTimeoutFlag == false))
  {
    xBeeTimeoutFlag = true;
    #ifdef PRINT_DEBUG
      Serial.println(F("No Rx data from XBee in 5 min"));
    #else
      strcpy(msgTweet, "Lost XBee communication.");
      SendTweet(msgTweet, poolData[P_POOL_TIME]);
    #endif
  }
  
  // Upload data to Xively
  if ((long)(millis() - xively_Upload_Timer) >= 0)
  {
    xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL;  // Reset timer
    SendDataToXively(); // Send Data to Xively
  }
  
  // Check inputs and send message (Tweet) if anything is wrong
  sendAlarmMessage();
  
  
  // Reboot if no successful updates in 30 minutes
  if((long) (millis() - xively_uploadTimout_timer) >= 0)
  {
    strcpy(msgTweet, "Reboot: can't upload to Xively.");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
    xively_uploadTimout_timer = millis() + XIVELY_UPDATE_TIMEOUT;
    delay(1000);
    software_Reset();
  }
  
  // If PRINT_DEBUG is on, then delay 1000 so you don't fill up the serial monitor too fast
#ifdef PRINT_DEBUG
  delay(1000);
#endif
  
} // loop()


// Check inputs and send out a tweet if anything is wrong
void sendAlarmMessage()
{

  // Twitter Flags (tf_) used so Twitter messages are only sent once
  static bool tf_highPresDrop;       // High pressure across filter
  static bool tf_highPressure;       // Pressure before filter
  static bool tf_lowPressure;        // Low pressure
  static bool tf_highAmps;           // High pump amps
  static bool tf_highPumpTemp;       // High pump temperature
  static bool tf_emergencyShutdown;  // Emergency shutdown flag
  static bool tf_pumpOnAtNight;      // Pump on at night
  static bool tf_waterFillOn;        // Water fill valve is on
  static bool tf_heaterIsOn;         // Heater is on
  // flags for sensor status.
  static bool tf_preHeatTemperatureSensor;
  static bool tf_postHeatTemperatureSensor;
  static bool tf_pumpTemperatureSensor;
  static bool tf_preFilterPresureSensor;
  static bool tf_postFilterPressureSensor;
  static bool tf_waterFillPressureSensor;
  static bool tf_pumpAmpsSensor;
  static bool tf_waterLevelSensor;
  

  
  
  char msgTweet[TWEETMAXSIZE];    // Holds text for twitter message.  Should be big enough for message and timestamp

  // If Arduino recently restarted (last 10 seconds), set waterFillCntDnFlag to true so it
  // doesn't send a tweet if the water fill is on
  if(millis() < 10000)
  { tf_waterFillOn = true; }
  
  
  // High pump amps
  if(PoolData[P_PUMP_AMPS] >= 17 && tf_highAmps == false)
  {
    tf_highAmps = true;
    sprintf(msgTweet, "High pump amps: %d.", (int) PoolData[P_PUMP_AMPS]);
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // High pump temperature
  if(PoolData[P_TEMP_PUMP] >= 175 && tf_highPumpTemp == false)
  {
    tf_highPumpTemp = true;
    sprintf(msgTweet, "High pump temp: %d.", (int) PoolData[P_TEMP_PUMP]);
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // High pressure
  if(PoolData[P_PRESSURE1] >= 40 && tf_highPressure == false)
  {
    tf_highPressure = true;
    sprintf(msgTweet, "High pump pressure: %d.", (int) PoolData[P_PRESSURE1]);
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // Check pressure drop across filter
  if((PoolData[P_PRESSURE1] > 25.0) && ((PoolData[P_PRESSURE1] - PoolData[P_PRESSURE2]) > 10.0) && (tf_highPresDrop == false))
  {
    tf_highPresDrop = true;
    strcpy(msgTweet, "High filter pressure.");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // Check low pressure Counter
  if(PoolData[P_LOW_PRES_CNT] >= 17 && tf_lowPressure == false)
  {
    tf_lowPressure = true;
    strcpy(msgTweet, "Low pressure fluctuations at pool pump.");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // Check for emergency shutdown and send tweet
  if(PoolData[P_CONTROLLER_STATUS] >= 4 && tf_emergencyShutdown == false)
  {
    char txtStatus[statusBufLen];
    tf_emergencyShutdown = true;
    strcpy(msgTweet, "Emergency Shutdown - ");
    controllerStatus(txtStatus,  PoolData[P_CONTROLLER_STATUS]);
    strcat(msgTweet, txtStatus);
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
    #ifdef PRINT_DEBUG
      Serial.println(msgTweet);
    #endif
  }
  
  // Send Tweet if water fill timer is opened
  if(PoolData[P_WATER_FILL_COUNTDN] > 1 && tf_waterFillOn == false)
  {
    // Wait a couple seconds in case water fill button is pressed a couple times, then read XBee data again
    delay(2500);
    uint16_t xbeeID;                // ID of transimitting xbee
    ReadXBeeData(&xbeeID);
    sprintf(msgTweet, "Water fill started for %d minutes.", (int) PoolData[P_WATER_FILL_COUNTDN]);
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
    tf_waterFillOn = true;  // flag so Tweet is only sent once
  }
  
  // Send Tweet of pump is running at night
  if(PoolData[P_PUMP_AMPS] > 6 && PoolData[P_POOL_TIME] >= 21 && tf_pumpOnAtNight == false)
  {
    tf_pumpOnAtNight = true;
    strcpy(msgTweet, "Dude, turn off the pool pump");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // Send tweet when heater comes on.  Don't want this every time, just once a day, so reset at night
  // Determine if Pool heater is on by comparing temperatures
  if(((PoolData[P_TEMP2] - PoolData[P_TEMP1]) > 5.0) && (PoolData[P_PUMP_AMPS] > 5.0) && tf_heaterIsOn == false)
  {
    tf_heaterIsOn = true;
    strcpy(msgTweet, "Pool heater is on");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  // Send tweet if any sensors are having trouble
  if ((sensorStatusbyte >> 0) & 1 == 0 && tf_preHeatTemperatureSensor == false )
  {
    tf_preHeatTemperatureSensor = true;
    strcpy(msgTweet, "Pool pre-heat temperature sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }

  if ((sensorStatusbyte >> 1) & 1 == 0 && tf_postHeatTemperatureSensor == false )
  {
    tf_postHeatTemperatureSensor = true;
    strcpy(msgTweet, "Pool post-heat temperature sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 2) & 1 == 0 && tf_pumpTemperatureSensor == false )
  {
    tf_pumpTemperatureSensor = true;
    strcpy(msgTweet, "Pool pump temperature sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 3) & 1 == 0 && tf_preFilterPresureSensor == false )
  {
    tf_preFilterPresureSensor = true;
    strcpy(msgTweet, "Pool pre-filter pressure sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 4) & 1 == 0 && tf_postFilterPressureSensor == false )
  {
    tf_postFilterPressureSensor = true;
    strcpy(msgTweet, "Pool post-filter pressure sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 5) & 1 == 0 && tf_waterFillPressureSensor == false )
  {
    tf_waterFillPressureSensor = true;
    strcpy(msgTweet, "Pool water fill pressure sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 6) & 1 == 0 && tf_pumpAmpsSensor == false )
  {
    tf_pumpAmpsSensor = true;
    strcpy(msgTweet, "Pool pump amps sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  if ((sensorStatusbyte >> 7) & 1 == 0 && tf_waterLevelSensor == false )
  {
    tf_waterLevelSensor = true;
    strcpy(msgTweet, "Pool water level sensor trouble");
    SendTweet(msgTweet, PoolData[P_POOL_TIME]);
  }
  
  
  // Reset one-shot message flags
  if(PoolData[P_PUMP_AMPS] < 10 && tf_highAmps == true)  // Reset high amps flag
  { tf_highAmps = false; }
  
  if(PoolData[P_TEMP_PUMP] < 110 && tf_highPumpTemp == true)  // Reset high pump temp flag
  { tf_highPumpTemp = false; }
  
  if(PoolData[P_PRESSURE1] < 110 && tf_highPressure == true)  // Reset high pressure flag
  { tf_highPressure = false; }
  
  if(PoolData[P_PRESSURE1] < 2 && tf_highPresDrop == true)  // Reset high pressure drop Flag
  { tf_highPresDrop = false; }
  
  if(PoolData[P_LOW_PRES_CNT] == 0 && tf_lowPressure == true) // Reset low pressure counter flag
  { tf_lowPressure = false; }
  
  if(PoolData[P_CONTROLLER_STATUS] < 4 && tf_emergencyShutdown == true)  // Reset emergency shutdown flag
  { tf_emergencyShutdown = false; }
  
  if(PoolData[P_WATER_FILL_COUNTDN] == 0 && tf_waterFillOn == true)  // Reset water fill timer flag
  { tf_waterFillOn = false; }

  if(PoolData[P_POOL_TIME] < 21 && tf_pumpOnAtNight == true)  // Reset pump running at night flag
  { tf_pumpOnAtNight = false; }

  if(PoolData[P_POOL_TIME] < 21 && tf_heaterIsOn == true)  // Reset heater on flag
  { tf_heaterIsOn = false; }

  
} // sendAlarmMessage


//=========================================================================================================
// Send twitter text, appends the time to the message to avoid twitter blocking duplicate messages
//=========================================================================================================
int SendTweet(char * txtTweet, double fpoolTime)
{
  
  char cpoolTime[19];   // char arry to hold pool time
  
  if(strlen(txtTweet) <= TWEETMAXSIZE - 20) // Make sure message there is room in character array for the timestamp
  {
    sprintf(cpoolTime, " Pool Time: %01d:%02d", int(floor(fpoolTime)), (int)((fpoolTime - floor(fpoolTime)) * 60));
    strcat(txtTweet, cpoolTime);          // Append pool time decimal format to the message
  }
  
  if (twitter.post(txtTweet))
  {
    // Specify &Serial to output received response to Serial.
    // If no output is required, you can just omit the argument, e.g.
    // int tweetStatus = twitter.wait();
    #ifdef PRINT_DEBUG
      int tweetStatus = twitter.wait(&Serial);
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
        Serial.print(F("Twitter failed : code "));
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
  
  #ifdef PRINT_DEBUG
    // Print Tweet to serial monitor
    Serial.print(F("Tweet: "));
    Serial.println(txtTweet);
  #endif
  
  xively_Upload_Timer = millis() + XIVELY_UPDATE_INTERVAL; // increase timer for Xively so it doesn't send right after Twitter
  
  
} // SendTweet()


//=========================================================================================================
// Send data to Xively
// xivelyData array is the same as the poolData array in loop function
// Before sending, check to make sure data is realistic.  If not, don't send
//======================================================================================================================================
bool SendDataToXively()
{
  
  
  if (gotNewData == true)
  {
    if(PoolData[P_PRESSURE1] < 40)  // check for a valid pressure
    { datastreams[0].setFloat(PoolData[P_PRESSURE1]); }  // 0 - Pre-filter pressure
    
    if(PoolData[P_PRESSURE2] < 40)  // check for a valid pressure
    { datastreams[1].setFloat(PoolData[P_PRESSURE2]); }  // 1- Post-Filter pressure
    
    if(PoolData[P_PRESSURE3] < 100)  // check for a valid pressure
    { datastreams[2].setFloat(PoolData[P_PRESSURE3]); }  // 2- water fill pressure
    
    if(PoolData[P_TEMP1] > 40 && PoolData[P_TEMP1] < 150)
    { datastreams[3].setFloat(PoolData[P_TEMP1]); }    // 3 - Pre heater temp
    
    if(PoolData[P_TEMP2] > 40 && PoolData[P_TEMP2] < 150)
    { datastreams[4].setFloat(PoolData[P_TEMP2]); }    // 4 - Post heater temp
    
    // Determine if Pool heater is on by comparing temperatures
    if(((PoolData[P_TEMP2] - PoolData[P_TEMP1]) > 5.0) && (PoolData[P_PUMP_AMPS] > 5.0))
    { datastreams[5].setInt(1); }              // 5 - Heater status On/Off
    else
    { datastreams[5].setInt(0); }
    
    if(PoolData[P_TEMP_PUMP] > 40 && PoolData[P_TEMP_PUMP] < 300)
    { datastreams[6].setFloat(PoolData[P_TEMP_PUMP]); } // 6 - Pump temp
    
    if(PoolData[P_PUMP_AMPS] < 50)
    { datastreams[7].setFloat(PoolData[P_PUMP_AMPS]); } // 7 - Pump amps
    
    datastreams[8].setInt((int) PoolData[P_LOW_PRES_CNT]); // 8 - Low pressure counter
    
    datastreams[9].setInt((int) PoolData[P_WATER_FILL_MINUTES]); // 9 - Mintues water fill valve was on today
    
    datastreams[13].setInt((int) PoolData[P_CONTROLLER_STATUS]); // 13 - Controller status code number
    
    datastreams[14].setInt(PoolData[P_LOW_WATER] ); // 14 - water level sensor - Calculated
    
    datastreams[15].setFloat(PoolData[P_WATER_LVL_BATT]/1000.0); // 15 - battery volts for water level sensor
    
  } // if gotNewData
  
  gotNewData = false; // reset got xbee data flag
  
  if(xBeeTimeoutFlag == true)
  { // No communication with XBEE
    Serial.println(F("No Xbee Comm"));
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
      flashLed(LED_XIVELY_SUCCESS, 1, 150);

      return true;
      break;
    case HTTP_ERROR_CONNECTION_FAILED:
      failures++;
      flashLed(LED_XIVELY_ERROR, 1, 150);
      #ifdef PRINT_DEBUG
        Serial.println(F("\nconnection to api.xively.com has failed. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_API:
      failures++;
      flashLed(LED_XIVELY_ERROR, 1, 150);
      #ifdef PRINT_DEBUG
        Serial.println(F("\nA method of HttpClient class was called incorrectly. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_TIMED_OUT:
      failures++;
      flashLed(LED_XIVELY_ERROR, 1, 150);
      #ifdef PRINT_DEBUG
        Serial.println(F("\nConnection with api.xively.com has timed-out. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    case HTTP_ERROR_INVALID_RESPONSE:
      failures++;
      flashLed(LED_XIVELY_ERROR, 1, 150);
      #ifdef PRINT_DEBUG
        Serial.println(F("\nInvalid or unexpected response from the server. Failures = "));
        Serial.println(failures);
      #endif
      return false;
      break;
    default:
      failures++;
      flashLed(LED_XIVELY_ERROR, 1, 150);
      #ifdef PRINT_DEBUG
        Serial.print(F("\nXively Unknown status: "));
        Serial.print(ret);
        Serial.print(F("  Failures = "));
        Serial.println(failures);
      #endif
      return false;
  }
}  // SendDataToXively()



//=========================================================================================================
//==================================================================================================================
bool ReadXBeeData(uint16_t *Tx_Id)
{
  
  // Read XBee data
  xbee.readPacket();
  
  if (xbee.getResponse().isAvailable())
  {
    // got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE )
    {
      // got a Rx packet
      xbee.getResponse().getRx16Response(rx16);  // I think this tells XBee to send the data over, not sure
      int dataLength = rx16.getDataLength();     // Get number of bytes of data sent from outside XBee
      uint16_t RxData[dataLength];               // Array to hold raw XBee data
      *Tx_Id = rx16.getRemoteAddress16();        // MY ID of Tx, remember MY is set as a hex number.  Useful if you have multiple transmitters
      
      // Convert data from bytes to integers, except last two bytes in array which are status bytes and don't get converted
      for(int i=0; i < dataLength; i=i+2)
      {
        RxData[i/2]  = rx16.getData(i) << 8;
        RxData[i/2] |= rx16.getData(i+1);
        PoolData[i/2] = (float) RxData[i/2] / 10.0; // value from XBee are 10x, convert back to normal size
      }
      
      // Put status bytes into byte variables
      sensorStatusbyte = PoolData[P_SENSORSTATUSBYTE];    // Each bit determines if sensor is operating properly
      ioStatusbyte = PoolData[P_IOSTATUSBYTE];            // I/O state of digital I/O
      
      gotNewData = true;
      xbeeTimeout = millis() + 300000UL; // Reset xbee timeout timer, 5 minutes
      xBeeTimeoutFlag = false;
     
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
      flashLed(LED_XBEE_ERROR, 1, 150);
      xbeeErrors++;
      return false;
    }
  }
  // XBee is not available
  else if (xbee.getResponse().isError())
  {
    // Got something, but not a packet
    // You get error code 0 when the other Xbee isn't sending anydata over
    #ifdef PRINT_DEBUG
      //srg      Serial.print(F("XBee error reading packet. Err code: "));
      //      Serial.println(xbee.getResponse().getErrorCode());
    #endif
    flashLed(LED_XBEE_ERROR, 1, 150);
    return false;
  } // End Got Something
  else
  {
    // xbee not available and no error
    #ifdef PRINT_DEBUG
      //srg      Serial.print(F("XBee not avail, Err code: "));
      //      Serial.println(xbee.getResponse().getErrorCode());
    #endif
    flashLed(LED_XBEE_ERROR, 1, 150);
    return false;
  }
  
} // ReadXbeeData()


//=========================================================================================================
// Print data sent from Pool Xbee
//=========================================================================================================
void PrintPoolData()
{
  static byte printheadcounter;
  
  // Print heading every 15 rows
  if (printheadcounter == 0)
  {
    Serial.println(F("\nTemp1\tTemp2\ttemp3\tAmps\tPres1\tPres2\tPres3\tP-Cnt\tWFmin\tbatt\tstat\tsesnsor\tI/O"));
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
  Serial.print(sensorStatusbyte, BIN);
  Serial.print(F("\t"));
  Serial.print(ioStatusbyte, BIN);
  Serial.println();
  
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
//  Flash LEDs to indicate if status of program
//=========================================================================================================
void flashLed(int pin, int times, int wait)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    
    if (i + 1 < times)
    { delay(50); }
  }
} // flashLed()



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


