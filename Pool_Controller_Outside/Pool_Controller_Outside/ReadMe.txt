Features to add:
- web enabled remote shutdown
- web enabled remote water fill
- Can you get rid of panStamps and just use xbee for low water level.  You'll need to figure out how to do a low power setup for the xbee
- Replace wire.h library with I2C (if you are keeping panStamp).  Be sure to disable pull-ups on I2C lines
- Use 3 momentary pushbuttons of On-Off-Auto.  You can use one analog input for all 3 plus water fill.
You can do all all this on A4, which is where water fill input goes now. If you want Auto and On button 
to light up,  use pins D12 & D13, they are the current inputs to on-off-auto.  Make provision for heater on and pool light on pushbuttons.
See: http://tronixstuff.com/2012/02/29/tutorial-analog-input-for-multiple-buttons-part-two/
Or use an I2C I/O expander like this: 296-26293-5-ND
- Turn heater on/off with arduino
- turn pool light on/off with arduino
- Bluefruit for programming wirelessly


PCB Ideas
Build CT circuit onto PCB
Terminal strips for all sensor connections
Build RTC on PCB
Small LCD to display status
1-5 volt pressure transducers should have a 25k pull-down resistor so you know when if cable has a problem
protect arduino against short circuits of sensor wire
Reverse voltage protection
Level shifter for I2C for panStamp
Pu  `t xbee in adafruit xbee adapter (https://www.youtube.com/watch?v=8DMZSxS-xVc&t=16m), this way it will be horizantel and signal might be stronger



Hardware:
Leonardo
Xbee
panStamp
RTC  http://www.adafruit.com/products/264
t/c amp  http://bit.ly/1iXdMhm
CT circuit: LM324, 2x 1n4148, 10uF cap, 220Ω, 62k, 1.5k
LCD - future
relay for water fill valve (draws 1/4 amp) Use reed relay 9007-05-00
relay for heter (draws 1/3 amp) use reed relay 9007-05-00


Terminal Blocks / JST plugs
Power 2-pin
One wire temp: 3-pin
Pres 1: 2-pin
Pres 2: 2-pin
Pres 3: 2-pin
Motor Amps: 2-pin
Water fill pushbutton: 2-pin
Water fill LED output: 2-pin
Water fill output: 2-pin
On-Off-Auto switch: 3-pin
Pump on output: 2-pin
Heater On output: 2-pin (future use)



Bugs:




==============================================================================================
This sketch monitor swimming pool conditions: Water temp, pump pressure
It will control the pump
It will transmit data via xbee to another arduino inside the house which will send info to the internet



XBee info
Source: http://code.google.com/p/xbee-arduino/
Xbee ver 0.3 wont compile unless you change NewSoftSerial.h to SoftwareSerial.h.
see: http://arduino.cc/forum/index.php?topic=84789.0
How to configure xBees http://code.google.com/p/xbee-api/wiki/XBeeConfiguration
xbee.h Documentation http://xbee-arduino.googlecode.com/svn/trunk/docs/api/index.html

Configure this Xbee with X-CTU (values are hex)
PAN 2323  Personal Area Network ID - all xBees need to be on same PAN
CH C      Channel, XBees on same network have to use the same channel
My 100    ID of xBee, all xBees need to be different.
CE 0      Sets this XBee as the and End Node
AP 2      API
DL 99    decimal Lower Byte Address (not used in 16-bit addressing)
Firmware 10E8


=== Hardware ===
Leonardo
Real Time Clock DS1307 from Adafruit http://www.adafruit.com/products/264  Battery: CR1220.  Uses I2C
Xbee Pro Series 1 -  XB24-AWI-001 (wire antenna)
Xbee shield from sparkfun http://www.sparkfun.com/products/9588, (switch away from edge to upload program)
Power Supply 12VDC, 1.2 Amps McMaster 7009K32 Omron S8VS-01512  http://www.ia.omron.com/product/item/s8vs0157h/index.html
Solid State Relay for pump McMaster 6323T13  CARLO GAVAZZI RJ1A60D50U http://www.gavazzionline.com/pdf/RJdatasheetmidi.pdf
Relay for Heater - heater draws 1/3 amp
Relay for Makeup Water - valve draws 1/4 amps, Solid State, Continental I.O.-ODC-R0-060,  McMaster # 8299K12
Therocouple probe for pump housing surface moount with adhesive McMaster 3648K34

Pressure Gauge - Grainger 2AGA4, Ashcroft 35X1009SD02L4FLXLJLL30#.  Manual http://wiki.goldthwaite.com/images/8/8e/Pool_Ashcroft_Pressure_Gauge.pdf
Pressure Transducer for pre-filter 0-30 PSI, Output 1-5VDC, Input 9-36V, Ashcroft G17M0215CD30#, Grainger 5LRX0
Pressure Transducer for water fill 0-100 PSI, Output 1-5VDC, Input 9-36V, Ashcroft G17M0215CD100#, Grainger 5LRX2 
Pressure transducers should be covered from rain and brough inside in the winter.  Use Quick disconnects to make for 
easy removal: McMaster 6537K37 & 6537K91
Nylon Liquid Tight cord grips for pressure transducers: McMaster 69915K51
Heat shrink tubing for pressure transducer cable, moisture sealing McMaster 7861K52
Waterproof connectors for pressure transducer, 4-conductor http://www.adafruit.com/products/744
Waterproof connector for pressure gauge 2-conductor http://www.adafruit.com/products/743 
Thermowells McMaster 3957K38 
Head for thermowells McMaster 38705K85 (I took heads off thermocouple probes)
Liquid tight for temp probes McMaster 7807K15 & 7529K402
Temp sensors: OneWire DS18B20
Dent 20 Amp Current Transformer CT‐HSC‐020‐U
DIN Mount Circuit breakers McMaster 7026K212 & 7026K223
Terminal blocks, DIN rail

Thermocouple Amplifier - www.adafruit.com MAX6675 breakout boards (discontinued May 2012)
For Pressure gauge (not pressure transducer)
1 243 Ω resistors

For Pump Amps from CT
1 MAX1044 voltage converter, Digikey MAX1044CPA+-ND
3 10uF electrolytic capacitors, Digikey P975-ND
1 Op Amp, rail-rail
2 1N4148 Diodes, Digikey 1N4148FS-ND
1 62K Ω resistor, Digikey P62.0KCACT-ND
1 1.5k Ω resistor, Digikey S1.5KCACT-ND
Water Valve for adding water to pool - sprinkler valve 24VAC. It will run on 9VDC, I'm using 12 VDC. Draws 1/4 amp at 9VDC.
DIN Rail enclosure for CT Amp circuit: OKW B6700120


== RTC ==
DS1307 chip
RTC Tutorial http://www.ladyada.net/learn/breakoutplus/ds1307rtc.html
To set time, un-comment line in setup(), upload sketch, then comment line out and upload agian.
This sets to time to the compile time.  If you don't resend the sketch with the command commented out, then if the
arduino reboots, it will reset to the compiled time
RTC.adjust(DateTime(__DATE__, __TIME__));

=== Wiring ===

Temperature
Adafruit MAX6675 breakout board pinout
--GND
--VCC          Red(-)--
--DO
--CS       Yellow(+) --
--CLK (SCK)

OneWire
-- GND
-- Data
-- +5V
4.7k resistor between data and 5v


Pre-Filter Pressure
Pressure Transducer: 1-5v output, 9-30 Vin
I measured the ADC at differet pressures and figured out formula using excel scatter plot
PSI = 0.0343 x ADC - 5.9077    R^2 = 0.9997
Don't use solid wires with transducer, it tends to break

Wiring:
Transducer Pin  Cables       Description
1              Red - Red      12 VDC
2              Blk - Blk      Common
3              Yel - Yel      Output (1-5 v)
4              Wht - Grn      Case ground (not connected)


Post-Filter Pressure
I measured the ADC at differet pressures and figured out formula using excel scatter plot
PSI = 0.0359 x ADC - 6.2548    R^2 = 0.9938

Pressure Gauge wiring.  Put 243 Ω resistor between Analog Input and Ground
Red: +12 volts
Blk: (Analog In) ----/\/\/\/\---- GND

CT measuring Amps:
Dent 20 Amp CT
Use this circuit to convert volts: http://forum.sparkfun.com/viewtopic.php?f=14&t=28775&start=30#p140890

Switch Inputs:
Pump swich has Auto - Off - Manual On settings.  Enable internal pull-ups and wire between input and ground
Water fill pushbotton - Turn on internal pull-up and wire between input and ground

Outputs:
Pump relay can be wired directly to Arduino pin
Relays for heater and water fill can be powered directly by Arduino.  Relays are optically isolated

Communication
This arduino uses I2C to communicate with panStamp RX which receives low water sensor status
I2C is also used to communicate with Real Time Clock
Since we need Wire.h library for the real time clock, it will be used to communicate with panStamp, normally I would prefer I2C.h library



I2C Packet structure
-------------------------------------
byte 0:     I2C Slave address (this panStamp is the slave)
byte 1:     TX panStamp Status: 255 = offline, 1 = online
byte 2:     Water Level 2 Min: 0 = level ok, 1 = level low, 2 = sensor offline
byte 3:     Water Level LIVE:  0 = level ok, 1 = level low
byte 4:     Is lid level: true/false
byte 5,6:   Accelerometer x-axis value
byte 7,9:   Accelerometer y-axis value
byte 9,10:  Accelerometer z-axis value
byte 11,12: Battery voltage
byte 13:    Reserved for water leaking inside sensor
byte 14:    Checksum


XBee packet structure (all integers)
-------------------------------------
0  Temp Pre Heater
1  Temp Post Heater
2  Temp Pump houseing
3  Pump Amps
4  Pressure pre-filter
5  Pressure post filter
6  Pressure Water Fill
7  Low pressure counter
8  Controller Status Number
9  Minutes of water added today
10 Is lid flat
11 Pool time
12 Water level sensor battery voltage
13 Low Water Level - calculated
14 Sensor Input Status Byte - sensorStatusbyte
15 Dicrete I/O status byte - ioStatusByte


Sensors status byte - sensorStatusbyte
1 = sensor is working properly, 0 = not working
-----------------------------------------------
0 Pre-heat temperature
1 Post-heat temperature
2 Pump temperature
3 Pre-filter pressure
4 Post-filter pressure
5 Water fill pressure
6 pump amps
7 Water level sensor


Discrete I/O status byte - ioStatusByte
shows on/off state of I/O
---------------------------
0 Pump on/off relay
1 Auto-Off-On switch is in Auto Position
2 Auto-Off-On switch is in On Position
3 Water fill LED
4 Water fill pushbutton input
5 Water fill valve relay
6 Heater on/off relay output
7 Real time Water Level Sensor 0 = water okay, 1 = water low
 



