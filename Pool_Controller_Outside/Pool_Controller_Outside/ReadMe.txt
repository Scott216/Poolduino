/*
 
 
 
 Features to add:
 -change WaterFillTime so it can handle millis rollover
 - web enabled remote shutdown
 - web enabled remote water fill
 -Test low pressure shutdown - the one for constant low pressure for 5 minutes.
 -Add small LCD screen to display status
 -Can you get rid of panstamps and just use Xbee for low water level.  You'll need to figure out how to do a low power setup for the xbee
 -See if you can use I2C.h library instead of wire.h because you can enable timeouts.  Problem is for the RTC you'll need conver this too so you're not using both wire.h and I2C.h at the same time
 -Got pressure shutdown when changing DE.  When on/off/auto is in off position, reset this timer
 
 PCB Ideas
 Leonardo plug into PCB
 Build CT circuit onto PCB
 Terminal strips for all sensor connections
 Build RTC on PCB, digikey DS1307+-ND, Crystal digikey X1124-ND
 If you do your own RTC refer see pg 6 of this doc: http://www.emesystems.com/pdfs/parts/DS1307_xtal.pdf it explains PCB requirements for the crystal
 Build MAX6675 onto PCB (sparkfun has these)
 Small LCD to display status
 Put RTC on dedicated I2C pins
 Use Panstamp to get water level, communicate with Leonardo on I2C
 If you need more I/O, use panstamp
 
 
 Bugs:
 
 
 ==============================================================================================
 This sketch monitor swimming pool conditions: Water temp, pump pressure
 It will control the pump, can turn the heater on/off and turn
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
 
 
 ==RTC==
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
 4              Wht - Grn     Case ground (not connected)
 
 
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
 byte 0: I2C Slave address
 byte 1: TX panStamp Status: 255 = offline, 0 = online
 byte 2: water level sensor 0 - level ok, 1 - level is low, 2 - sensor offline
 byte 3: low water sensor (real time), True - low water, False - level okay, ignores accelerometer
 byte 4: Level Lid: false - lid is not level
 byte 5: battery voltage LSB
 byte 6: battery voltage MSB
 
 
