
This Arduino receives data from the outside pool controller and uploads it to Xively
http://xively.com/feeds/65673/workbench
It uses an XBee series 1



To do:
Add OLED display
See if you can update water added today every minute, not just when it's complete it's cycle
Don't seem to need xbeeID Tx_Id parameter in xbee function
Add a CRC or checksum to xbee data packet
  http://forum.arduino.cc/index.php?topic=45742.0
  

Figure out how to send commands to outside xbee from inside xbee, then see if you can shut pump off from iphone.  Make sure there is confirmation between xbees.
  Turn pump off from iphone/website http://www.yaler.org/
  Instructables for Yaler and Arduino http://www.instructables.com/id/Arduino-Web-LED/step5/Accessing-and-controlling-the-Arduino-from-the-Web/


========================================================================================
xBee Series 1 Receiver - this is the COORDINATOR

Source: http://code.google.com/p/xbee-arduino/
Xbee ver 0.3 wont compile unless you change NewSoftSerial.h to SoftwareSerial.h.
see: http://arduino.cc/forum/index.php?topic=84789.0
How to configure xBees http://code.google.com/p/xbee-api/wiki/XBeeConfiguration
xbee.h Documentation http://xbee-arduino.googlecode.com/svn/trunk/docs/api/index.html

Configure this Xbee with X-CTU (values are hex)
PAN 2323  Personal Area Network ID - all xBees need to be on same PAN
CH C      Channel, XBees on same network have to use the same channel
My 250    ID of xBee, all xBees need to be different.
CE 1      Enable Coordinator - sets this XBeef as the Coordinator
AP 2      API
DL 90    decimal Lower Byte Address (not used in 16-bit addressing)
Firmware 10E8

Hardware
Arduino Mega (Uno doesn't have enough RAM)
XBee Series 1 Pro, chip antenna
Xbee shield from sparkfun http://www.sparkfun.com/products/9588, (switch away from edge to upload program)


xively Streams
http://xively.com/feeds/65673/workbench
0 Pressure before filter
1 Pressure after filter
2 Pressure drop across filter
3 Temp before heater
4 Temp after heater
5 Heater Status
6 Temp pump housing
7 Pump Amps
8 Low pressure counter
9 Water fill minutes today
10 Status: Pump off, pump on, water fill on, swithch off, emergency shutdown
11 successes
12 failures

Status code numbers from outside controller
0 - pump off - normal
1 - pump on
2 - adding water
3 - Pump off - manual (via pump switch)
4 - emergency pump off - low pressure - fluctuations
5 - emergency pump off - low pressure - continiously low for 5 minutes
6 - emergency pump off - high amps
7 - emergency pump off - high pump temp
8 - emergency pump off - shutdown from web


To send data to this particular xbee, the transmitter uses the Receiver's MY address, not the DL address
See http://www.digi.com/support/kbase/kbaseresultdetl?id=2187

XBee packet structure
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

