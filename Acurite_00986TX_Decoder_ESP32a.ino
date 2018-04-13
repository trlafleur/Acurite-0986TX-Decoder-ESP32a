
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
 *  the device when it sends the 2nd block. (The last bit is NOT sent
 *  correctly if the last bit of the CRC is a 1. So for now we don’t
 *  decode the 2nd message.)
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
 * 8 measured hi and lo edges in a row, 4 high and 4 low, of 
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
 *  
 *  MQTT
 *    A message sent to this device by topic: SUBSCRIBE_TOPIC, with a "R"
 *     in the 1st byte will reset all Min/Max settings
 *     
 *    Temperature is sent to MQTT server if using a ESP32 or ESP8266 processor
 *     
 *  MQTT Data Sent:
 *    Temperature, Min, Max and Battery Status for both devices
 *     Alarms for over-temperature and Low Battery
 *    
 *  E-Mail:
 *    If enable, alarms are also send via E-mail or SMS
 *     http://www.emailtextmessages.com/
 *  
 *  Integration time for alarms can be set for each sensor.     
 *  
 *  Radio:
 *   Using an RXB6 or equivalent, connect to 3.3v, gnd and connect dataout
 *    to interrupt pin on CPU.
 *    
 *   RFM69 connect DIO-2 to interrupt pin on CPU.
 *    
 *   Antenna is 17.2cm long at 433MHz
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
 *  10-Apr-2018 1.0e  TRL - Added E-Mail support
 *  12-Apr-2018 1.0f  TRL - Added I2C OLE display, removed Moteino Mega
 *  
 *  
 *  Notes:  1)  Tested with Arduino 1.8.5
 *          2)  Testing with a 433Mhz RFM69 
 *                RFM69OOK lib from https://github.com/kobuki/RFM69OOK
 *                DIO2 connected to pin interrupt pin.
 *          3)  Tested with a RXB6 receiver connected to interrupt pin.
 *          4)  Tested using a TTGO R1 ESP32 module
 *          5)  ESP32 and ESP8266 supported sending data via MQTT and E-Mail
 *          6)  ESP8266 tested with a NodeMCU 1.0
 *          7)  Added E-mail-SMS Support NOTE: You must edit Gsender.h with your info
 *          8)
 *          
 *  Todo:   1) Fix issues with RFM69 receiver, work in progress, not working
 *          2) move MyDebug define's inside processor type
 *          3) 
 *          4) 
 *          5) 
 * 
 * Tom Lafleur --> tom@lafleur.us
 * 
 */

 /* ************************************************************* */
#define VERBOSE_OUTPUT
//#define DISPLAY_BIT_TIMING
//#define DISPLAY_DATA_BYTES
#define MyDEBUG
#define IF_MQTT
#define IF_EMAIL
//#define RFM69
#define OLED U8X8_SSD1306_128X64_NONAME_HW_I2C            // OLED-Display on board


#ifdef RFM69
  #include <RFM69OOK.h>
  #include <SPI.h>
  #include <RFM69OOKregisters.h>
  RFM69OOK radio;
#endif

// Ring buffer size has to be large enough to fit
// data and sync signal, at least 2x the number of bit we expect
// so 44 * 2 = 88,  round up to a power of 2, --> 128 for now
#define RING_BUFFER_SIZE  128

#define SYNC_MAX              2600          // Sync high time
#define SYNC_MIN              1300          // Sync low time (last sync pulse is ~1300us)

#define PULSE_BIT1            900           // Bit 1 time in us
#define PULSE_BIT0            500           // Bit 0 time in us
#define PULSE_TOLL            120           // +- Tolerance for bit timming
#define PRE_PULSE             190           // On pulse prior to data

#define PULSE_SHORT_NOISE     PRE_PULSE     // anything shorter that this is noise, Pre pulse is ~200US
#define PULSE_LONG_NOISE      SYNC_MAX      // anything longer that this is noise


// On the Arduino connect the data pin, the pin that will be 
// toggling with the incomming data from the RF module, to
// a pin that can be configured for interrupt 
// on change, change to high or low.


#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>         // http://arduino.cc/en/Reference/Wire ??
#include "Gsender.h"
#include "MovingAverage.h"

// OLED Display 
#include <U8g2lib.h>      // https://github.com/olikraus/u8g2

/* ************************************************************* */
// Select processor includes
#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
  #include <esp_wps.h>
  #include <WiFiClientSecure.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiClientSecure.h>
#endif

#define SKETCHNAME    "Started, Acu-rite 0986 Decoder, "
#define SKETCHVERSION "Ver: 1.0f"

// create an instance of WiFi client 
  WiFiClient espClient;

  // WiFi information
  // change it with your ssid-password
  const char* ssid = "MySSID";                 // <----------- Change This
  const char* password = "MyPass";       // <----------- Change This
  
#ifdef IF_EMAIL
   const char* MySendToAddress = "MyEmail";                                 // <----------- Change This for E-Mail
// const char* MySendToAddress = "MyPhone@vtext.com";                                  // <----------- Change This for SMS
//      For SMS format, see -->   http://www.emailtextmessages.com/
#endif

  uint8_t connection_state = 0;                    // Connected to WIFI or not
  uint16_t reconnect_interval = 10000;             // If not connected wait time to try again
  
#ifdef IF_MQTT
    #include <PubSubClient.h>
   
    // MQTT Server IP Address or FQDN
    const char* mqtt_server = "192.168.167.32";    // <----------- Change This
  
    // create an instance of PubSubClient client 
    PubSubClient client(espClient);
    
    // My topics
    #define RTEMP_TOPIC     "RSF/REF/Temp"                                           // <----------- Change These as needed
    #define FTEMP_TOPIC     "RSF/FRZ/Temp"
    #define RBATT_TOPIC     "RSF/REF/BATT"
    #define FBATT_TOPIC     "RSF/FRZ/BATT"
    #define RALARM_TOPIC    "RSF/REF/ALARM"
    #define FALARM_TOPIC    "RSF/FRZ/ALARM"
    #define BALARM_TOPIC    "RSF/BATT/ALARM"
    #define RMIN_TOPIC      "RSF/REF/MIN"
    #define RMAX_TOPIC      "RSF/REF/MAX"
    #define FMIN_TOPIC      "RSF/FRZ/MIN"
    #define FMAX_TOPIC      "RSF/FRZ/MAX"
    #define SUBSCRIBE_TOPIC "RSF/REF/RESET"
#endif    // End of: #ifdef IF_MQTT


// Hardware pin definitions for TTGOv1 Board with OLED SSD1306 I2C Display
#define OLED_RST 16         // ESP32 GPIO16 (Pin16) -- SD1306 Reset
#define OLED_SCL 15         // ESP32 GPIO15 (Pin15) -- SD1306 Clock
#define OLED_SDA 4          // ESP32 GPIO4  (Pin4)  -- SD1306 Data


// create an instance for OLED Display
#ifdef OLED
    OLED u8x8(OLED_RST, OLED_SCL, OLED_SDA);
#else
   U8X8_NULL u8x8;
#endif

// create an instance for Moving Average
  MovingAverage <int> REF (7);        // create a moving average over last n values   // <----------- Change This as needed
  MovingAverage <int> FRZ (7);        // 10 * 120 sec = 1200sec = 20min

  
  #define MAX_RTEMP     45            // Max temperature for refrigerator             // <----------- Change This as needed
  #define MAX_FTEMP     20            // Max temperature for freezer
  
  #define AlarmTimeToWait          120L            // Wait this amount of time for next alarm message, in Minutes  // <----------- Change These as needed
  #define BattAlarmTimeToWait     1440L            // Wait this amount of time for next alarm message, in Minutes
  
  unsigned long LastTimeRef  = 0;
  unsigned long LastTimeFrz  = 0;
  unsigned long LastTimeBatt = 0;
  bool R_Flag = false;
  bool F_Flag = false;
  bool B_Flag = false;
     
  unsigned long currentMillis = 0;            // a 1 Minute clock timer
  unsigned long interval = 60000;             // = 60 sec --> 1 Minure
  unsigned long previousMillis = 0;
  unsigned long Minute = 0;
  
  unsigned long BlockFailCounter  = 0;
  unsigned long CRCFailCounter    = 0;
  
  char msg[60];                               // char string buffer



/* ************************************************************* */
#ifdef ARDUINO_ARCH_ESP32
  /* pin that is attached to interrupt */
  #define DATAPIN          12                 // interrupt pin
  byte interruptPin = DATAPIN;
  #define MyInterrupt (digitalPinToInterrupt(interruptPin))
  
  #define MyLED             2
  
  // define below are use in debug as trigers to logic analyzer
  #define MySync            36                // Trigger on Sync found
  #define MyBit             37                // Trigger on bit edge
  #define MyFrame           38                // Trigger at end of frame


/* ************************************************************* */
#elif ARDUINO_ARCH_ESP8266

  #define DATAPIN            D1                // D1 is interrupt
  byte interruptPin = DATAPIN;
  #define MyInterrupt (digitalPinToInterrupt(interruptPin))
 
  // Note: their are two LED on the NodeMCU Rev1 board
  // D0-->16 on the board and D4-->2 on the ESP12 that is connected to U1-TXD
  #define MyLED             16
  
  // define below are use in debug as trigers to logic analyzer
  //#define MySync            D0                // Trigger on Sync found
  //#define MyBit             D1                // Trigger on bit edge
  //#define MyFrame           D2                // Trigger at end of frame
  
#else
  #error CPU undefined.....
#endif


 /* ************************************************************* */
#define SYNCPULSECNT      4                   // 4 sync pulses (8 edges)
#define SYNCPULSEEDGES    (SYNCPULSECNT *2 )

#define DATABYTESCNT      5                   // Number of bytes to look for 
#define DATABITSCNT       (DATABYTESCNT * 8)  // Number of bits to look for
#define DATABITSEDGES     (DATABITSCNT * 2)   // Number of edges to look for

// The pulse durations are measured in micro seconds between
// pulse edges.
unsigned long pulseDurations[RING_BUFFER_SIZE];   // where we store the pulse edges
unsigned int syncIndex  = 0;                // index of the last bit time of the sync signal
unsigned int dataIndex  = 0;                // index of the first bit time of the data bits (syncIndex+1)
bool         syncFound = false;             // true if sync pulses found
bool         received  = false;             // true if enough sync pulses bits are found
unsigned int changeCount = 0;               // Count of pulses edges

unsigned char dataBytes[DATABYTESCNT];      // Decoded data storage
unsigned long mytime = 0;                   // event time
signed char temp = 0;                       // temperature in deg F
int RMinTemp = 127;                         // Max temp is 127deg
int RMaxTemp = 0;
int FMinTemp = 127;                         // Max temp is 127deg
int FMaxTemp = 0;


/* ************************************************************* */
#ifdef OLED
void init_display(void) 
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clear();
    u8x8.setFlipMode(1);
}
#endif // OLED


#ifdef IF_MQTT
/* ************************************************************* */
  void receivedCallback(char* topic, byte* payload, unsigned int length) 
  {
    Serial.println("Message received: ");
    Serial.println(topic);
  
    Serial.print("payload: ");
    for (int i = 0; i < length; i++) 
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == 'R') 
    {
      RMinTemp = 127;
      RMaxTemp = 0;
      FMinTemp = 127;
      FMaxTemp = 0;
    }
  }


/* ************************************************************* */
  void mqttconnect() 
  {
    /* Loop until reconnected */
    while (!client.connected()) 
    {
      Serial.println();
      Serial.print("MQTT connecting to: ");
      Serial.println (mqtt_server);
      
      /* client ID */
       String clientId = WiFi.macAddress();                // use our MAC address as MQTT Client ID
   
      /* connect now */
      if (client.connect(clientId.c_str()))                  
      {
        Serial.print("MQTT connected, Client ID: ");
        Serial.println(clientId);
        /* subscribe topic with default QoS 0*/
       client.subscribe(SUBSCRIBE_TOPIC);
      } else 
      {
        Serial.print("failed, status code =");
        Serial.print(client.state());
        Serial.println("try again in 5 seconds");
        /* Wait 5 seconds before retrying */
        delay(5000);
      }
    }   // End of: while (!client.connected()) 
  }   // End of:  mqttconnect() 

#endif    // End of: IF_MQTT


/* ************************************************************* */
uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
   static uint16_t attempt = 0;
   WiFi.disconnect(); 
   Serial.print("Connecting WiFi to: ");
   if(nSSID) {
       WiFi.begin(nSSID, nPassword);  
       Serial.println(nSSID);
   } else {
       WiFi.begin(ssid, password);
       Serial.println(ssid);
   }

   uint8_t i = 0;
   while(WiFi.status()!= WL_CONNECTED && i++ < 50)
   {
       delay(200);
       Serial.print(".");
   }
   ++attempt;
   Serial.println("");
   if(i == 51) {
       Serial.print("Connection: TIMEOUT on attempt: ");
       Serial.println(attempt);
       WiFi.disconnect(); 
       return false;
   }
   Serial.println("Connection: ESTABLISHED");
   Serial.print  ("Got IP address: ");
   Serial.println(WiFi.localIP());
   return true;
}


/* ************************************************************* */
void Awaits()
{
   uint32_t ts = millis();
   while(!connection_state)
   {
       delay(200);
       if(millis() > (ts + reconnect_interval) && !connection_state)
       {
           connection_state = WiFiConnect();
           ts = millis();
       }

     ESP.restart();     // <---------------- experiment 
   }
}


 /* ************************************************************* */
 /*
 * code to print formatted hex 
 */
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
   char tmp[length*2+1];
   byte first;
   int j = 0;
   for (uint8_t i = 0; i < length; i++) 
     {
       first = (data[i] >> 4) | 48;
       if (first > 57) tmp[j] = first + (byte)39;
       else tmp[j] = first ;
       j++;
    
       first = (data[i] & 0x0F) | 48;
       if (first > 57) tmp[j] = first + (byte)39; 
       else tmp[j] = first;
       j++;
     }
   tmp[length*2] = 0;
   Serial.print("0X");
   Serial.print(tmp);
}


/* ************************************************************* */
// reverses all bits in a byte
uint8_t reverse8(uint8_t x) 
{
    x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}


/* ************************************************************* */
// CRC8 for little endian format
uint8_t crc8le(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init) 
{
    uint8_t crc = init, i;
    unsigned byte;
    uint8_t bit;
    
    for (byte = 0; byte < nBytes; ++byte) 
    {
      for (i = 0x01; i & 0xff; i <<= 1) 
        {
            bit = (crc & 0x80) == 0x80;
            if (message[byte] & i) 
            {
              bit = !bit;
            }
              crc <<= 1;
              if (bit) {
              crc ^= polynomial;
            }
        }
    crc &= 0xff;
    }
    return reverse8(crc);
}

/* ************************************************************* */
/*
 * Look for the sync pulse train, 4 high-low pulses of
 * 1600uS high and 1500uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses
 * approximately 1600 uS long.
 * T0 is the high pulse, T1 is the low pulse
 */
bool isSync(unsigned int idx) 
{
   // check if we've received 4 sync pulses of correct timing
   for( int i = 0; i < SYNCPULSEEDGES; i += 2 )
   {
      unsigned long t1 = pulseDurations[(idx+RING_BUFFER_SIZE-i) % RING_BUFFER_SIZE];
      unsigned long t0 = pulseDurations[(idx+RING_BUFFER_SIZE-i-1) % RING_BUFFER_SIZE];      
      
      // if any of the preceeding 8 pulses are out of bounds, short or long,
      // return false, if no sync found
      if( t0 < (SYNC_MIN) || t0 > (SYNC_MAX) ||  t1 < (SYNC_MIN)  || t1 > (SYNC_MAX) )
        {
           return false;
        }
   }
   return true;
}


/* ************************************************************* */
/* Interrupt  handler 
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the Arduino LED on each interrupt. 
 */
void interrupt_handler() 
{
   volatile static unsigned long duration = 0;
   volatile static unsigned long lastTime = 0;
   volatile static unsigned int ringIndex = 0;
   volatile static unsigned int syncCount = 0;
   volatile static unsigned int bitState  = 0;

   // Ignore if we haven't finished processing the previous 
   // received signal in the main loop.
   if( received == true ) {return;}       // return, we are not finish with processor last block

   bitState = digitalRead (DATAPIN);
   digitalWrite(MyLED, bitState);         // LED to show receiver activity

   // calculating timing since last change
   long time = micros();
   duration = time - lastTime;
   lastTime = time;

   // Known errors in bit stream are runt's --> short and long pulses.
   // If we ever get a really short, or really long 
   // pulse's we know there is an error in the bit stream
   // and should start over.
   if ( (duration > (PULSE_LONG_NOISE)) || (duration < (PULSE_SHORT_NOISE)) )    // This pulse must be noise...  
   {
      received = false;
      syncFound = false;
      changeCount = 0;                                  // restart, start looking for data bits again
   }

   // if we have good data, store data in ring buffer
   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
   pulseDurations[ringIndex] = duration;
   changeCount++;                                       // found another edge

#ifdef MyDEBUG
      digitalWrite(MyBit, !digitalRead(MyBit) );        // LED to show we have a bit
     // digitalWrite (MyBit, LOW);          
     // delayMicroseconds (50);
     // digitalWrite (MyBit, HIGH);
#endif

   // detected sync signal
   if( isSync (ringIndex) )                              // check for sync on each bit received
   {
      syncFound = true;
      changeCount = 0;                                   // lets restart looking for data bits again
      syncIndex = ringIndex;
      dataIndex = (syncIndex + 1) % RING_BUFFER_SIZE;

#ifdef MyDEBUG
      digitalWrite(MySync, !digitalRead(MySync) );        // LED to show we have sync
//      digitalWrite (MySync, LOW);          
//      delayMicroseconds (50);
//      digitalWrite (MySync, HIGH);
#endif    
    }
    
   // If a sync has been found, then start looking for the
   //  data bit edges in DATABITSEDGES
   if( syncFound )
   {       
      // not enough bits yet, so no full message block has been received yet
      if( changeCount < DATABITSEDGES )            
        { received = false; }
      
      else 
      
      if( changeCount >= DATABITSEDGES )            // check for too many bits
        {      
          changeCount = DATABITSEDGES;              // lets keep bits we have, CRC will kill this block if bad
          detachInterrupt(MyInterrupt);             // disable interrupt to avoid new data corrupting the buffer
          received = true;   
        }
           
#ifdef MyDEBUG
        digitalWrite(MyFrame, !digitalRead(MyFrame) );         // LED to show that we have block
//        digitalWrite (MyFrame, LOW); 
//        delayMicroseconds (100);
//        digitalWrite (MyFrame, HIGHƒ);
#endif  
      

    }    // end of if syncFound
}    // end of interrupt_handler


const char compile_date[]  = __DATE__ ", " __TIME__;

/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */
void setup()
{
   Serial.begin(115200);
   delay(2000);
   
   Serial.println("");
   Serial.print(SKETCHNAME);
#ifdef RFM69
   Serial.println("RFM69");
#else
   Serial.println("External Receiver");
#endif
  Serial.println (SKETCHVERSION);
  Serial.println (compile_date);
  Serial.println("");
 
   pinMode(DATAPIN, INPUT);             // data interrupt pin set for input
   pinMode(MyLED, OUTPUT);              // LED output
   digitalWrite (MyLED, LOW);

#ifdef MyDEBUG
   pinMode(MySync, OUTPUT);              // sync bit output
   digitalWrite (MySync, LOW);
   
   pinMode(MyBit, OUTPUT);               // data bit output
   digitalWrite (MyBit, LOW);
   
   pinMode(MyFrame, OUTPUT);             // end of frame bit output
   digitalWrite (MyFrame, LOW);
#endif


#ifdef RFM69
    pinMode( 14, INPUT);        // RFM69 RST
    //digitalWrite (14, LOW);
    pinMode(26, INPUT);         // DIO-0
    pinMode(33, INPUT);         // DIO-1
    pinMode(32, INPUT);         // DIO-2  This is where we get the RX data --> comnnected to INT-0
    radio.initialize();
    //radio.setBandwidth(OOK_BW_10_4);
    radio.setRSSIThreshold(-70);
    radio.setFixedThreshold(20);
    radio.setSensitivityBoost(SENSITIVITY_BOOST_HIGH);
    radio.setFrequencyMHz(433.92);
    radio.receiveBegin();
#endif


// Setup WiFi
// WiFi.config(ip, dns, gateway, subnet);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  connection_state = WiFiConnect();
  if(!connection_state)                     // if not connected to WIFI
       Awaits();                            // constantly trying to connect

    delay (1000);


#ifdef IF_MQTT
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked 
  when client received subscribed topic */
  client.setCallback(receivedCallback);
#endif    // End of: #ifdef IF_MQTT


#ifdef OLED
// initialize display  
    init_display();     
    u8x8.setCursor(0,1);
    u8x8.drawString(0, 0,"0986 Decoder");
    u8x8.printf("%d.%d.%d.%d",WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
#endif

   pinMode(MyInterrupt, INPUT_PULLUP);
   attachInterrupt(MyInterrupt, interrupt_handler, CHANGE);
   
}   // end of setup


/* ************************************************************* */
/*
 * Convert pulse durations to bits.
 * 
 */
int convertTimingToBit(unsigned int t0, unsigned int t1) 
{
   if( t1 > (PULSE_BIT1-PULSE_TOLL) && t1 < (PULSE_BIT1+PULSE_TOLL) )
    { return 1; }

   else if ( t1 > (PULSE_BIT0-PULSE_TOLL) && t1 < (PULSE_BIT0+PULSE_TOLL) )
    { return 0; }
   
   return -1;                   // error, if undefined bit timimg
}



/* ************************************************************* */
// 0986TX send's a meassge every ~120 sec, so lets average temperature
// over a number of sample, if is greater that our alarm settings, we need to send
// an alarm, but only once every so many minutes.

void MaxRefrigeratorAlarm (int temp)
{ 
  if (R_Flag == false)                         // see if this is 1st time here for this alarm...
   {
#ifdef IF_MQTT      
      snprintf (msg, 6, "%d", temp);
      client.publish (RALARM_TOPIC, msg);
#endif
#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
     String subject = "RSF Refrigerator Alarm!";
     snprintf (msg, 60, "Alarm set at: %dF,  Temperature is: %dF", MAX_RTEMP, temp);
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) {
         Serial.println("E-mail Message sent.");
     } else {
         Serial.print("Error, sending message: ");
         Serial.println(gsender->getError());
     }

#endif
      Serial.println ("Refrigerator Alarm"); 
          
      R_Flag = true;
      LastTimeRef = Minute;                   // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeRef + AlarmTimeToWait ) )       // see if it time to re-send alarm
      { R_Flag = false; }                                   // Yes, reset alarm flag
  }
}


/* ************************************************************* */
void MaxFreezerAlarm(int temp)
{
  if (F_Flag == false)                         // see if this is 1st time here for this alarm...
   {
#ifdef IF_MQTT
      snprintf (msg, 6, "%d", temp);
      client.publish (FALARM_TOPIC, msg);
#endif

#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
     String subject = "RSF Freezer Alarm!";
     snprintf (msg, 60, "Alarm set at: %dF,  Temperature is: %dF", MAX_FTEMP, temp);
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) {
         Serial.println("E-Mail Message sent.");
     } else {
         Serial.print("Error sending message: ");
         Serial.println(gsender->getError());
     }

#endif
      Serial.println ("Freezer Alarm");    
      F_Flag = true;
      LastTimeFrz = Minute;                    // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeFrz + AlarmTimeToWait ) )       // see if it time to re-send alarm
      { F_Flag = false; }                                   // Yes, reset alarm flag
  }
}


/* ************************************************************* */
void BatteryLowAlarm (int device)
{
  if (B_Flag == false)                         // see if this is 1st time here for this alarm...
   {
      if (device == 1) {
        snprintf (msg, 50, "Battery Low, Refrigerator Sensor: %d", device);
        Serial.println ("Battery Low Alarm 1R");  
      }
      if (device == 2) {
        snprintf (msg, 50, "Battery Low, Frezzer Sensor: %d", device);
        Serial.println ("Battery Low Alarm 2F");  
      }
#ifdef IF_MQTT
      client.publish (BALARM_TOPIC, msg);
#endif

#ifdef IF_EMAIL
     Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
     String subject = "RSF Low Battery Alarm!";
     if(gsender->Subject(subject)->Send(MySendToAddress, msg)) {
         Serial.println("E-Mail Message sent.");
     } else {
         Serial.print("Error sending message: ");
         Serial.println(gsender->getError());
     }
#endif
      B_Flag = true;
      LastTimeBatt = Minute;                    // save the current time
   }
  else
  {
    if ( Minute >= (LastTimeBatt + BattAlarmTimeToWait ) )  // see if it time to re-send alarm
          { B_Flag = false; }                               // Yes, reset alarm flag
  }
}


/* ************************************************************* */
void MQTT_Send (void)
{
#ifdef IF_MQTT
// send sensor number, 1R or 2R temperature and battery status, and alarm status to MQTT server

          // get temperature, -temperature is in Sign and magnitude format
          if (dataBytes[0] >= 128) { temp =  (~dataBytes[0] & 0x7f); temp = (temp | 0x80) + 1; }
          else { temp =  dataBytes[0]; }
          snprintf (msg, 6, "%d", temp);
              
          if ( dataBytes[3] & 0x01)                                       // Sensor 2, Frezzer
             { 
              client.publish (FTEMP_TOPIC, msg);                          // send temperature

              u8x8.setCursor(0,3);
              u8x8.clearLine(3)
              u8x8.printf("Frz Temp: %dF",temp );

              if (temp > FMaxTemp) {FMaxTemp = temp;}
              if (temp < FMinTemp) {FMinTemp = temp;}
              
              snprintf (msg, 6, "%d", FMinTemp);
              client.publish (FMIN_TOPIC, msg);                           // send min temperature              
              snprintf (msg, 6, "%d", FMaxTemp);
              client.publish (FMAX_TOPIC, msg);                           // send max temperature 
                            
              if ( (dataBytes[3] & 0x02) == 0x02) 
                {
                  client.publish (FBATT_TOPIC, "Low Battery 2F");
                  BatteryLowAlarm ( 2 );
                }
              
              int Frz_Temp = FRZ.CalculateMovingAverage((int) temp);
              if (Frz_Temp >= MAX_FTEMP) { MaxFreezerAlarm(Frz_Temp); }   // do we have an alarm? Yes
              else { F_Flag = false; }    // no alarm now
              }
          else                                                           // Sensor 1, Refrigerator 
             { 
              client.publish (RTEMP_TOPIC, msg);

              u8x8.setCursor(0,5);
              u8x8.clearLine(5)
              u8x8.printf("Ref Temp: %dF",temp );
              
              if (temp > RMaxTemp) {RMaxTemp = temp;}
              if (temp < RMinTemp) {RMinTemp = temp;}
              
              snprintf (msg, 6, "%d", RMinTemp);
              client.publish (RMIN_TOPIC, msg);                           // send min temperature              
              snprintf (msg, 6, "%d", RMaxTemp);
              client.publish (RMAX_TOPIC, msg);                           // send max temperature               
              
              if (dataBytes[3] & 0x02 == 0x02) 
                {
                  client.publish (RBATT_TOPIC, "Low Battery 1R");
                  BatteryLowAlarm ( 1 );
                }
              int Ref_Temp = REF.CalculateMovingAverage((int) temp);
              if (Ref_Temp >= MAX_RTEMP)  { MaxRefrigeratorAlarm(Ref_Temp); }
              else { R_Flag = false; }   // no alarm now
              }   
#endif
}   // End of MQTTSend


/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */
/*
 * Main Loop
 * Wait for received to be true, meaning a sync stream plus
 * all of the data bit edges have been found.
 * Convert all of the pulse timings to bits and calculate
 * the results.
 */
void loop()
{

#ifdef IF_MQTT 
  /* if client was disconnected then try to reconnect again */
   if (!client.connected()) {
     mqttconnect();
   }
  /* this function will listen for incomming 
  subscribed topic-process-invoke receivedCallback */
   client.loop();
#endif  


// lets setup a long duration timer at 1 minute tick
  currentMillis = millis ();                                  // get current time
  if (currentMillis - previousMillis >= interval)
        {previousMillis = currentMillis; Minute++;}           // add one to minute couter if time..

 
   if( received == true )                                     // check to see if we have a full block of bits to decode
   {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(MyInterrupt);
      
      unsigned int ringIndex;
      bool fail = false;


/* ************************************************************* */
// Print the bit stream for debugging. 
// Generates a lot of chatter, normally disable this. 
// T0 is the length of the up pulse, or pre-pulse, T1 is the down pulse, or data pulse   
#ifdef DISPLAY_BIT_TIMING
      Serial.println("");
      Serial.print("syncFound = ");
      Serial.println(syncFound);
      Serial.print("changeCount = ");
      Serial.println(changeCount);

      Serial.print("syncIndex = ");
      Serial.println(syncIndex);

      Serial.print("dataIndex = ");
      Serial.println(dataIndex);

      ringIndex = (syncIndex - (SYNCPULSEEDGES-1)) % RING_BUFFER_SIZE;

      for ( int i = 0; i < (SYNCPULSECNT + DATABITSCNT); i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex % RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE] );

         Serial.print("bit ");
         Serial.print( i );
         Serial.print(" =  ");
         Serial.print(bit);
         Serial.print(", t0 = ");
         Serial.print(pulseDurations [ringIndex % RING_BUFFER_SIZE]);
         Serial.print(", t1 = ");
         Serial.println(pulseDurations [(ringIndex + 1) % RING_BUFFER_SIZE]);

         ringIndex += 2;
      }
#endif // endif of: DISPLAY_BIT_TIMING


/* ************************************************************* */
// Build a byte array with the raw data received

      fail = false;                             // reset bit decode error flag

      // clear the data bytes array
      for( int i = 0; i < DATABYTESCNT; i++ )    { dataBytes[i] = 0; }
        
      ringIndex = (syncIndex +1 ) % RING_BUFFER_SIZE;

      for( int i = 0; i < DATABITSCNT; i++ )
      {
         int bit = convertTimingToBit ( pulseDurations[ringIndex % RING_BUFFER_SIZE], 
                                          pulseDurations[(ringIndex +1 ) % RING_BUFFER_SIZE] ); 
                                                                          
         if( bit < 0 )                 // check for a valid bit, ie: 1 or zero, -1 = error
           {  
              fail = true;
              break;                    // exit loop
           }
         else
           {
              dataBytes[i/8] |= bit << ( 7 - (i % 8));   // pack into a byte
           }        
         ringIndex += 2;
      }

#ifdef DISPLAY_DATA_BYTES

      if (fail )
        { Serial.println("Data Byte Display --> Decoding error."); }
      else
        {
          for( int i = 0; i < DATABYTESCNT; i++ )
            {
              PrintHex8 (&dataBytes[i], 1);
              Serial.print(",");
            }
          Serial.print("  ");
  
          for( int i = 0; i < DATABYTESCNT; i++ )
            {
              Serial.print(dataBytes[i], BIN);
              Serial.print(",");
            }
          Serial.print("  ");
        }

#endif  // end of: DISPLAY_DATA_BYTES


/* ************************************************************* */
// lets extract data from the sensor
// all data bytes are now in dataBytes[DATABYTESCNT]
// Bits are received LSB first, so we need to reverse order of all of the bits

    for (unsigned char i=0; i <= 4; i++)  { dataBytes[i] = reverse8(dataBytes[i]); }

/* ************************************************************* */
        if (!fail)                                                // if fail, we decoded some bits wrong, so don't process this block
        {
          if ( crc8le (dataBytes, 4, 0x07, 0) == dataBytes[4] )    // if CRC8 is good... 
          {

 #ifdef VERBOSE_OUTPUT            

              Serial.print(Minute);                                 // Time stamp for display
              Serial.print(" ");    
              Serial.print("Acurite 986 sensor: 0X");
              Serial.print( dataBytes[1], HEX );
              Serial.print( dataBytes[2], HEX );
              
              // print sensor number, 1 or 2
              if ( dataBytes[3] & 0x01) 
                { Serial.print(" - 2F: "); }      
              else 
                { Serial.print(" - 1R: "); }
              
              // print battery status
              if (dataBytes[3] & 0x02) { Serial.print(" low battery " );}
              
              // print temperature
              if (dataBytes[0] >= 128) { temp =  (~dataBytes[0] & 0x7f); temp = (temp | 0x80) + 1; }
              else { temp =  dataBytes[0];}
              
              Serial.print ( temp, DEC);
              Serial.println(" F");
               
  #ifdef MyDEBUG
              Serial.print (Minute);         
              Serial.print (" Block Error Counter: ");                   // Print CRC and Block decode errors counters
              Serial.print (BlockFailCounter);
              Serial.print (", CRC Error Counter: ");
              Serial.println (CRCFailCounter);
              
  #endif    //  End of MyDEBUG
             
#endif      // VERBOSE_OUTPUT           

          // Send data to MQTT server
              MQTT_Send ();                           // sending MQTT messages

         
            }   // End of if (crc8le...
          else  
            {
              
    #ifdef MyDEBUG
              Serial.print (Minute);                                    // Print CRC8 information
              Serial.print ( " CRC: is: ");
              Serial.print ( dataBytes[4], HEX );
              Serial.print ( " Should be: ");
              Serial.println ( crc8le(dataBytes, 4, 0x07, 0), HEX);     // I know, this is a wast of time to do this again...
    #endif
              CRCFailCounter++;                                         // if CRC is bad, keep count
              }
          
        }   // End of if (!fail)...
        
        else 
          { BlockFailCounter++; }                                  // if block decode is bad, keep count
          
      received = false;
      syncFound = false;
      
      delay (250);       // this will eliminate 2nd block of data if its sent
      
      // re-enable interrupt
      attachInterrupt (MyInterrupt, interrupt_handler, CHANGE);
      
   }    // end of:  if( received == true )

}   // end of: loop



