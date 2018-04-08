
/**********************************************************************
 * Arduino code to decode the Acurite 00986TX wireless temperature sensor
 *
 * The 00986TX wireless temperature probe contains a 433.92 MHz
 *  wireless transmitter. The temperature from the sensor is
 *  sent approximately every 120 seconds. Their are two sensor
 *  1R is for refrigerator, 2F is for freezer
 *
 * The 00986TX sends a block of data, 4 SYNC pulse + a DATA stream
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
 *  Temperature is sent to MQTT server if using a ESP32 processor
 *  
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
 *
 *  Notes:  1)  Tested with Arduino 1.8.5
 *          2)  Testing using Moteino Mega Rev4 with 433Mhz RFM69 
 *                RFM69OOK lib from https://github.com/kobuki/RFM69OOK
 *                DIO2 connected to pin INT0
 *          3)  Tested with a RXB6 receiver connected to pin INT0
 *          4)  Tested using a TTGO R1 ESP32 module
 *          5)  ESP32 version supports sending data via MQTT
 *          6)
 *          
 *  Todo:   1) Fix issues with RFM69 receiver, work in progress, not working
 *          2) move MyDebug define's inside processor type
 *          3) Improve WiFi connection and retry... not very robust at this point
 *          4) Add improved interrupt robustnest 
 *          5)
 * 
 * Tom Lafleur --> tom@lafleur.us
 * 
 */

 /* ************************************************************* */

//#define RFM69
#define VERBOSE_OUTPUT
//#define DISPLAY_BIT_TIMING
//#define DISPLAY_DATA_BYTES
//#define MyDEBUG

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

#define SYNC_HIGH             1700          // Sync high time
#define SYNC_LOW              1400          // Sync low time (last sync pulse is ~1300us)
#define SYNC_TOLL             650           // +- Tolerance for sync timming

#define PULSE_BIT1            900           // Bit 1 time in us
#define PULSE_BIT0            550           // Bit 0 time in us
#define PULSE_TOLL            100           // +- Tolerance for bit timming
#define PRE_PULSE             200           // On pulse prior to data

#define PULSE_SHORT_NOISE     PRE_PULSE-50            // anything shorter that this is noise, Pre pulse is ~200US
#define PULSE_LONG_NOISE      SYNC_HIGH+SYNC_TOLL+1   // anything longer that this is noise


// On the arduino connect the data pin, the pin that will be 
// toggling with the incomming data from the RF module, to
// digital pin 10. Pin D10 is interrupt 0 and can be configured
// for interrupt on change, change to high or low.

// The squelch pin in an input to the radio that squelches, or
// blanks the incoming data stream. Use the squelch pin to 
// stop the data stream and prevent interrupts between the 
// data packets if desired. (not used at this time)


/* ************************************************************* */
// Select processor options
#ifdef ARDUINO_ARCH_ESP32       // ESP32 TTgo V1

#include <WiFi.h>
#include "esp_wps.h"
#include <PubSubClient.h>
#include "MovingAverage.h"

// WiFi and MQTT information
// change it with your ssid-password
const char* ssid = "MySSID";                  // <----------- Change This
const char* password = "MYPASS";              // <----------- Change This
// MQTT Server IP Address or FQDN
const char* mqtt_server = "192.168.20.32";    // <----------- Change This

// create an instance of WiFi and PubSubClient client 
WiFiClient espClient;
PubSubClient client(espClient);
MovingAverage <int> REF(7);       // create a moving average over last n values <----------- Change These as needed
MovingAverage <int> FRZ(7);       //  7 * 120 sec = 840sec = ~14 min

#define MAX_RTEMP     40          // Max temperature for refrigerator <----------- Change This as needed
#define MAX_FTEMP     25          // Max temperature for freezer      <----------- Change This as needed

// My topics
#define RTEMP_TOPIC     "RSF/REF/Temp"                                <----------- Change These as needed
#define FTEMP_TOPIC     "RSF/FRZ/Temp"
#define RBATT_TOPIC     "RSF/REF/BATT"
#define FBATT_TOPIC     "RSF/FRZ/BATT"
#define RALARM_TOPIC    "RSF/REF/ALARM"
#define FALARM_TOPIC    "RSF/FRZ/ALARM"

#define AlarmTimeToWait    60L              // Wait this amount of time for next alarm message, in Minutes <----------- Change This as needed

unsigned long LastTimeRef = 0;
unsigned long LastTimeFrz = 0;
bool R_Flag = false;
bool F_Flag = false;
   
unsigned long currentMillis = 0;            // a 1 Minute clock timer
unsigned long interval = 60000;             // = 60 sec --> 1 Minure
unsigned long previousMillis = 0;
unsigned long Minute = 0;

char msg[30];                               // char string buffer

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* pin that is attached to interrupt */
#define DATAPIN          12                 // interrupt pin
byte interruptPin = DATAPIN;
#define MyInterrupt (digitalPinToInterrupt(interruptPin))

#define SQUELCHPIN       4                  // option for some receivers

#define MyLED            2

// define below are use in debug as trigers to logic analyzer
#define MySync            15                // Trigger on Sync found
#define MyBit             13                // Trigger on bit edge
#define MyFrame           14                // Trigger at end of frame

/* ************************************************************* */
#elif __AVR_ATmega1284P__      // MoteinoMega LoRa

#define DATAPIN          10                // D10 is interrupt 0 on a Moteino Mega
#define MyInterrupt       0
#define SQUELCHPIN        4
#define MyLED            15

// define below are use in debug as trigers to logic analyzer
#define MySync            12                // Trigger on Sync found
#define MyBit             13                // Trigger on bit edge
#define MyFrame           14                // Trigger at end of frame

/* ************************************************************* */
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
unsigned int temp = 0;                      // temperature in deg F


#ifdef ARDUINO_ARCH_ESP32
 /* ************************************************************* */
void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
}

 /* ************************************************************* */
void mqttconnect() {
  /* Loop until reconnected */
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
  //    client.subscribe(LED_TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

#endif


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
 */
bool isSync(unsigned int idx) 
{
   // check if we've received 4 sync pulses of correct timing
   for( int i = 0; i < SYNCPULSEEDGES; i += 2 )
   {
      unsigned long t1 = pulseDurations[(idx+RING_BUFFER_SIZE-i) % RING_BUFFER_SIZE];
      unsigned long t0 = pulseDurations[(idx+RING_BUFFER_SIZE-i-1) % RING_BUFFER_SIZE];    
      
      // if any of the preceeding 8 pulses are out of bounds, short or long,
      // return false, no sync found
      if( t0 < (SYNC_HIGH-SYNC_TOLL) || t0 > (SYNC_HIGH+SYNC_TOLL) ||
          t1 < (SYNC_LOW-SYNC_TOLL)  || t1 > (SYNC_LOW+SYNC_TOLL) )
      {
         return false;
      }
   }
   return true;
}


/* ************************************************************* */
/* Interrupt  handler 
 * Tied to pin 10 INT0 of Moteino Mega
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
   if( received == true ) {return;}

   bitState = digitalRead(DATAPIN);
   digitalWrite(MyLED, bitState);         // LED to show receiver activity

   // calculating timing since last change
   long time = micros();
   duration = time - lastTime;
   lastTime = time;

   // Known errors in bit stream is are runt/short and long pulses.
   // If we ever get a really short, or really long 
   // pulse's we know there is an error in the bit stream
   // and should start over.
   if ( (duration > (PULSE_LONG_NOISE)) || (duration < (PULSE_SHORT_NOISE)) )    // This must be noise...  
   {
      received = false;
      syncFound = false;
      changeCount = 0;                          // restart, start looking for data bits again
   }

   // if we have good data, store data in ring buffer
   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
   pulseDurations[ringIndex] = duration;
   changeCount++;                               // found another edge

#ifdef MyDEBUG
      digitalWrite (MyBit, HIGH);          
      delayMicroseconds (1);
      digitalWrite (MyBit, LOW);
#endif

   // detected sync signal
   if( isSync (ringIndex) )                       // check for sync
   {
      syncFound = true;
      changeCount = 0;                            // lets restart looking for data bits again
      syncIndex = ringIndex;
      dataIndex = (syncIndex + 1) % RING_BUFFER_SIZE;

#ifdef MyDEBUG
      digitalWrite (MySync, HIGH);          
      delayMicroseconds (1);
      digitalWrite (MySync, LOW);
#endif
     
    }

   // If a sync has been found, then start looking for the
   //  data bit edges in DATABITSEDGES
   if( syncFound )
   {       
      // not enough bits yet, so no full message has been received yet
      if( changeCount < DATABITSEDGES )            
        { received = false; }
      
      else 
      
      if( changeCount >= DATABITSEDGES )            // check for too many bits
        {      
          changeCount = DATABITSEDGES;              // lets keep bits we have, CRC will kill if bad
          detachInterrupt(MyInterrupt);             // disable interrupt to avoid new data corrupting the buffer
          received = true;   
        }
        else
        {
           detachInterrupt(MyInterrupt);            // disable interrupt to avoid new data corrupting the buffer
           received = true;
           
#ifdef MyDEBUG 
        digitalWrite (MyFrame, HIGH); 
        delayMicroseconds (100);
        digitalWrite (MyFrame, LOW);
#endif  
      
        }   // end of else
    }    // end of if syncFound
}    // end of interrupt_handler


/* ************************************************************* */
void setup()
{
   Serial.begin(115200);
   delay(2000);
   Serial.println("");
   Serial.print("Started 00986 Decoder, ");
#ifdef RFM69
    Serial.println("RFM69");
#else
   Serial.println("External Receiver");
#endif

   pinMode(DATAPIN, INPUT);             // data interrupt pin set for input
   pinMode(MyLED, OUTPUT);              // LED output

   pinMode(SQUELCHPIN, OUTPUT);         // data squelch pin on radio module
   digitalWrite(SQUELCHPIN, HIGH);      // UN-squelch data

#ifdef MyDEBUG
   pinMode(MySync, OUTPUT);              // sync bit output
   digitalWrite (MySync, LOW);
   
   pinMode(MyBit, OUTPUT);               // data bit output
   digitalWrite (MyBit, LOW);
   
   pinMode(MyFrame, OUTPUT);             // end of frame bit output
   digitalWrite (MyFrame, LOW);
#endif


#ifdef ARDUINO_ARCH_ESP32       // ESP32 TTgo V1
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
 
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    esp_wifi_wps_start(0);
  }

  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked 
  when client received subscribed topic */
  client.setCallback(receivedCallback);

#elif __AVR_ATmega1284P__      // MoteinoMega LoRa
#ifdef RFM69
  pinMode( 3, INPUT);         // RFM69 RST
  //digitalWrite (3, LOW);
  pinMode( 2, INPUT);         // DIO-0
  pinMode(22, INPUT);         // DIO-1
  pinMode(21, INPUT);         // DIO-2  This is where we get the RX data --> comnnected to INT-0
  radio.initialize();
  //radio.setBandwidth(OOK_BW_10_4);
  radio.setRSSIThreshold(-70);
  radio.setFixedThreshold(20);
  radio.setSensitivityBoost(SENSITIVITY_BOOST_HIGH);
  radio.setFrequencyMHz(433.92);
  radio.receiveBegin();
#endif

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



#ifdef ARDUINO_ARCH_ESP32       // ESP32 TTgo V1

/* ************************************************************* */
// 0986TX send's a meassge every ~120 sec, so lets average temperature
// over a number of sample, if is greater that our alarm settings, we need to send
// an alarm, but only once every so many minutes.
void MaxRefrigeratorAlarm (int temp)
{ 
  if (R_Flag == false)                         // see if this is 1st time here for this alarm...
   {
      snprintf (msg, 6, "%d", temp);
      client.publish (RALARM_TOPIC, msg);
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
      snprintf (msg, 6, "%d", temp);
      client.publish (FALARM_TOPIC, msg);
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
#endif


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

#ifdef ARDUINO_ARCH_ESP32       // ESP32 TTgo V1
/* if client was disconnected then try to reconnect again */
   if (!client.connected()) {
     mqttconnect();
   }
/* this function will listen for incomming 
  subscribed topic-process-invoke receivedCallback */
   client.loop();
#endif

// lets setup a long duration timer at 1 minute 
  currentMillis = millis ();                                // get current time
  if (currentMillis - previousMillis >= interval)
        {previousMillis = currentMillis; Minute++;}
 
   if( received == true )                     // check to see if we have a full block of bits to decode
   {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(MyInterrupt);
      unsigned int ringIndex;
      bool fail = false;

/* ************************************************************* */
// Print the bit stream for debugging. 
// Generates a lot of chatter, normally disable this.    
#ifdef DISPLAY_BIT_TIMING
      Serial.print("syncFound = ");
      Serial.println(syncFound);
      Serial.print("changeCount = ");
      Serial.println(changeCount);

      Serial.print("syncIndex = ");
      Serial.println(syncIndex);

      Serial.print("dataIndex = ");
      Serial.println(dataIndex);

      ringIndex = (syncIndex - (SYNCPULSEEDGES-1)) % RING_BUFFER_SIZE;

      for( int i = 0; i < (SYNCPULSECNT + DATABITSCNT); i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex % RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex + 1) % RING_BUFFER_SIZE] );

         Serial.print("bit ");
         Serial.print( i );
         Serial.print(" = ");
         Serial.print(bit);
         Serial.print(" t0 = ");
         Serial.print(pulseDurations [ringIndex % RING_BUFFER_SIZE]);
         Serial.print(" t1 = ");
         Serial.println(pulseDurations [(ringIndex + 1) % RING_BUFFER_SIZE]);

         ringIndex += 2;
      }
#endif // endif of DISPLAY_BIT_TIMING


/* ************************************************************* */
// Build a byte array with the raw data received

      fail = false;                             // reset bit decode error flag

      // clear the data bytes array
      for( int i = 0; i < DATABYTESCNT; i++ )
         { dataBytes[i] = 0; }
        
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

      if( fail )
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

#endif  // end of DISPLAY_DATA_BYTES


/* ************************************************************* */
// lets extract data from the sensor
// all data bytes are now in dataBytes[DATABYTESCNT]
// Bits are received LSB first, so we need to reverse order of all of the bits

    for (unsigned char i=0; i <= 4; i++)  { dataBytes[i] = reverse8(dataBytes[i]); }

/* ************************************************************* */
#ifdef VERBOSE_OUTPUT 

          if ( crc8le(dataBytes, 4, 0x07, 0) == dataBytes[4] )
          {
              signed char temp;
              //Serial.println(""); 
              mytime = millis();
              Serial.print(mytime/1000);
              Serial.print(" ");    
              Serial.print("Acurite 986 sensor: 0X");
              Serial.print( dataBytes[1], HEX );
              Serial.print( dataBytes[2], HEX );
              
              // print sensor number, 1 or 2
              if ( dataBytes[3] & 0x01) { Serial.print(" - 2F: "); }      
              else { Serial.print(" - 1R: "); }
              
              // print battery status
              if (dataBytes[3] & 0x02) { Serial.print(" low battery " );}
              
              // print temperature
              if (dataBytes[0] >= 128) { temp =  (~dataBytes[0] & 0x7f); temp = (temp | 0x80) + 1; }
              else { temp =  dataBytes[0];}
              
              Serial.print ( temp, DEC);
              Serial.println(" F"); 

#ifdef MyDEBUG
              
              Serial.print(mytime/1000);
              Serial.print( " CRC Error: is: ");
              Serial.print( dataBytes[4], HEX );
              Serial.print( " Should be: ");
              Serial.println( crc8le(dataBytes, 4, 0x07, 0), HEX);
#endif
           }
             
#endif      // VERBOSE_OUTPUT           


#ifdef ARDUINO_ARCH_ESP32       // ESP32 TTgo V1

// send sensor number, 1R or 2R temperature and battery status, and alarm status to MQTT server

          // get temperature, -temperature is in Sign and magnitude format
          if (dataBytes[0] >= 128) { temp =  (~dataBytes[0] & 0x7f); temp = (temp | 0x80) + 1; }
          else { temp =  dataBytes[0]; }
          snprintf (msg, 6, "%d", temp);
              
          if ( dataBytes[3] & 0x01) 
             { 
              client.publish (FTEMP_TOPIC, msg);
              if (dataBytes[3] & 0x02 == 0x02) client.publish (FBATT_TOPIC, "Low Battery 2F");
              int Frz_Temp = FRZ.CalculateMovingAverage((int) temp);
              if (Frz_Temp >= MAX_FTEMP) { MaxFreezerAlarm(Frz_Temp); }     // do we have an alarm? Yes
              else { F_Flag = false; }    // no alarm now
              }
          else 
             { 
              client.publish (RTEMP_TOPIC, msg);
              if (dataBytes[3] & 0x02 == 0x02) client.publish (RBATT_TOPIC, "Low Battery 1R");
              int Ref_Temp = REF.CalculateMovingAverage((int) temp);
              if (Ref_Temp >= MAX_RTEMP)  { MaxRefrigeratorAlarm(Ref_Temp); }
              else { R_Flag = false; }   // no alarm now
              }     
#endif
        
      received = false;
      syncFound = false;
      
      delay (250);       // this will eliminate 2nd block of data if its sent
      
      // re-enable interrupt
      attachInterrupt (MyInterrupt, interrupt_handler, CHANGE);
      
   }    // if receive is true

}   // end of loop



