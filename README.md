# Acurite 0986 decoder for: ESP32 ESP8266
A decoder for an Acurite 0968 refrigerator freezer sensor, ESP32, MQTT, Email

Work on this was stoped in favor of: Acurite-00592TX-Decoder-ESP32a

TX-RX range of this devices was poor once install in a refrigerator or freezer

Code works, but could use some of the improvements made in 00592 decoder.

~~~
/**********************************************************************
 * Arduino code to decode the Acurite 0986TX wireless temperature sensor
 *
 * The 0986TX wireless temperature probe contains a 433.92 MHz
 *  wireless transmitter. The temperature from the sensor is
 *  sent approximately every 120 seconds. Their are two sensor
 *  1R is for refrigerator, 2F is for freezer
 *
 * The 0986TX sends a block of data, 4 SYNC pulse + a DATA stream
 *  per temperature reading. Sometimes its sends one block of data,
 *  at other time its send two block. Their seems to be a bug in
 *  the device when it sends the 2nd block. The last bit is NOT sent
 *  correctly if the last bit of the CRC is a 1. So for now we don’t
 *  decode the 2nd message
 *
 * The sensor then sends 4 data sync pulses of approximately 50% 
 *  duty cycle. The sync pulses start with a 
 *  high level and continue for 4 high / low pulses.
 *
 * The data bits immediately follow the fourth low of the data
 *  sync pulses. 
 *
 * 1 bit ~900us low, preceded by a ~200us high pre pulse
 * 0 bit ~500us low, preceded by a ~200us high pre pulse
 *
 * 4 sync pulse, followed by 40 bits of data
 * 
 * 5 bytes block, sent once or twice, with a single short pulse between blocks
 *
 * Data is TX'ed in OOK/AM/CW on-off-keying format, a 1 = low, 0 = high or TX on
 * 
 * Data Format - 5 bytes, sent LSB first!
 *
 * TT ID ID SS CC
 *
 * T  - Temperature in Fahrenheit, integer, MSB = sign.
 *      Encoding is "Sign and magnitude"
 * ID - 16 bit sensor ID
 *      changes at each sensor power up
 * S  - status/sensor type
 *       0x00 = Sensor 1       refrigerator
 *       0x01 = Sensor 2       freezer
 *       0x02 = low battery
 * C  = CRC (CRC-8 poly 0x07, little-endian)
 * 
 * The code below works by receiving a level change interrupt 
 *  on each changing edge of the data stream from the RF module
 *  and recording the time in uSec between each edge.
 *
 * 8 measured hi and lo pulses in a row, 4 high and 4 low, of 
 *  approximately 1500 uSec each constitute a sync stream.
 *
 * The remaining 40 bits of data, or 80 edges, are measured
 *  and converted to 1s and 0s by checking the high to low
 *  pulse times. Noise is removed if pulse are too short, or 
 *  too long
 *
 * As a note: often the 1st sync pulse high is ~2300us long, last one is
 *  ~1300us long.
 *  
 *  Temperature is sent to MQTT server if using a ESP32 or ESP8266 processor
 *  
 *  MQTT
 *    A message sent to this device by topic: SUBSCRIBE_TOPIC, with a "R"
 *     in the 1st byte will reset all Min/Max settings
 *     
 *  MQTT Data Sent:
 *    Temperature, Min, Max and Battery Status for both devices
 *    Alarms for over-temperature and Low Battery
 *    
 *  Integration time for alarms can be set for each sensor     
 *  
 * *********************************************************************
 * Ideas on decoding protocol and prototype code from
 * Ray Wang (Rayshobby LLC) http://rayshobby.net/?p=8998
 *
 * Code based on Ray Wang's humidity _display.ino source.
 *
 * *********************************************************************
 * 
 CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  04-Apr-2018 1.0a  TRL - First Build
 *  05-Apr-2018 1.0b  TRL - Added support for RFM69-433Mhz receiver
 *  07-Apr-2018 1.0c  TRL - Added support for ESP32 and MQTT
 *  08-Apr-2018 1.0d  TRL - Added Min/Max reporting and resetting via MQTT
 *  
 *  
 *  Notes:  1)  Tested with Arduino 1.8.5
 *          2)  Testing using Moteino Mega Rev4 with 433Mhz RFM69 
 *                RFM69OOK lib from https://github.com/kobuki/RFM69OOK
 *                DIO2 connected to pin interrupt pin.
 *          3)  Tested with a RXB6 receiver connected to pin interrupt pin.
 *          4)  Tested using a TTGO R1 ESP32 module
 *          5)  ESP32 version supports sending data via MQTT
 *          6)  ESP8266 tested with a NodeMCU 1.0
 *          
 *  Todo:   1) Fix issues with RFM69 receiver, work in progress, not working
 *          2) move MyDebug define's inside processor type
 *          3) Improve WiFi connection and retry... not very robust at this point
 *          4) 
 * 
 * Tom Lafleur --> tom@lafleur.us
 * 
 */
~~~
