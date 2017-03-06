/*
 *  Ver 1.3m  4 Mar 2017 TRL 
 *  
 *  A program to control an APEX Destiny 6100(AN) Alarm panel. 
 *   This is running on a MoteinoMEGA, so we have lots of memory available
 *   Radio is 915MHz RFM95 LoRa
 *   Tested with MySensor 2.1.1 
 *   Tested with D6100 firmware 8.07
 *   
 *  Max message size from D6100 is 96 + 10 bytes, so we must have buffers to capture this.
 *
 *  In the D6100, we need to set location: 0155 to 009 to enable the serial port
 *  
 *  To receive System Notification Reports, we need to set locations 0387 to 0435 as needed, by adding a 016 
 *  to each value, if old value was 129, new would be 145. (See page 49 of manual)
 *
 *  Message format is documented in: Destiny 6100 RS-232 Interface Specification, Release 5.0, dated 9/1998
 *  Format of Messages:
 *  
 *  NNMSDDDDDD...DD00CS CR-LF
 *    NN = Message length
 *    MS = Message ann sub message type
 *    DD = hex data, two bytes per value
 *    00 = Reserved
 *    CS = Modulo 256 Checksum
 *    CR = Carage Return \r
 *    LF = Line Feed \n
 * 
 * CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  xx-May-2016       TRL - D6100g build routines for setting time
 *  xx-May-2016       TRL - D6100h added debug print
 *  xx-Dec-2016       TRL - D6100i added support of RFM60_ATC, and new text message for alarm zone type
 *  26-Nov-2016 1.2j  TRL - Changed radio to RFM95 LoRa
 *  20-Dec-2016 1.3k  TRL - Changed radio to RFM95 LoRa, make it work
 *  20-Dec-2016 1.3l  TRL - Changed radio freq to 928.5Mhz
 *  04-Mar-2017 1.3m  TRL - Added code to set time on D6100
 *  
 *
 *  Notes:  1)  Tested with Arduino 1.8.1
 *          2)  Testing using MoteinoMega LoRa Rev1 with RFM95
 *          3)  MySensor 2.1.1 30 Dec 2016
 *
 * Todo:
 *  done --> get time from MQTT and set time in D6100 
 *  done --> some cleanup of code
 *  fix TX LED issue
 *  If debug is on, then sending time to D6100 work, if off, sets wrong time... very odd!!
 *  
 */

/* ************************************************************************************** */
#include <string.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
/* ************************************************************************************** */
// Most of these items below need to be prior to #include <MySensor.h> 

/*  Enable debug prints to serial monitor on port 0 */
#define MY_DEBUG            // used by MySensor
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RFM95 
#define MY_DEBUG1           // used in this program, level 1 debug
//#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME    "Alarm D6100"
#define SKETCHVERSION "1.3m RFM95"

/* ************************************************************************************** */
// Enable and select radio type attached, coding rate and frequency
#define MY_RADIO_RFM95

/*  
 *   Pre-defined MySensor radio config's
 *   
 * | CONFIG           | REG_1D | REG_1E | REG_26 | BW    | CR  | SF   | Comment
 * |------------------|--------|--------|--------|-------|-----|------|-----------------------------
 * | BW125CR45SF128   | 0x72   | 0x74   | 0x04   | 125   | 4/5 | 128  | Default, medium range SF7
 * | BW500CR45SF128   | 0x92   | 0x74   | 0x04   | 500   | 4/5 | 128  | Fast, short range     SF7
 * | BW31_25CR48SF512 | 0x48   | 0x94   | 0x04   | 31.25 | 4/8 | 512  | Slow, long range      SF9
 * | BW125CR48SF4096  | 0x78   | 0xC4   | 0x0C   | 125   | 4/8 | 4096 | Slow, long range      SF12
 */
 
#define MY_RFM95_MODEM_CONFIGRUATION    RFM95_BW125CR45SF128
#define MY_RFM95_TX_POWER               23 // max is 23
//#define MY_RFM95_ATC_MODE_DISABLED
#define MY_RFM95_ATC_TARGET_RSSI        (-60)
#define MY_RFM95_FREQUENCY              (928.5f)


#ifdef __AVR_ATmega1284P__        // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
// MoteinoMEGA
//#define MY_RFM95_RST_PIN        RFM95_RST_PIN
#define MY_RFM95_IRQ_PIN          2
#define MY_RFM95_SPI_CS           4
#define MY_DEFAULT_TX_LED_PIN     14   // the PCB, on board LED
#define MY_DEFAULT_ERR_LED_PIN    12
#define MY_DEFAULT_RX_LED_PIN     13
#define MY_WITH_LEDS_BLINKING_INVERSE
#else
  #error Wrong processor selected
#endif

/* ************************************************************************************** */
// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE

#define MY_NODE_ID            5                 // My Sensor Node ID
#define MY_PARENT_NODE_ID     0                 // GW ID
#define CHILD_ID              1                 // ID of my Alarm sensor child

/* ************************************************************************************** */
/* These are use for local debug of code, hwDebugPrint is defined in MyHwATMega328.cpp */
#ifdef MY_DEBUG1
#define debug1(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug1(x,...)
#endif

#ifdef MY_DEBUG2
#define debug2(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug2(x,...)
#endif

/* ************************************************************************************** */
/* All #define above need to be prior to #include <MySensors.h> below */
#include <MySensors.h>                            // this need to be after most of the above settings

/* ************************************************************************************** */
unsigned long WatchDog_FREQUENCY = 300000;         // time to refresh gateway with alarm status ~1min = 60000, 5min = 300,000

MyMessage VAR1Msg         (CHILD_ID,V_VAR1);      // 24 Zone number
MyMessage VAR2Msg         (CHILD_ID,V_VAR2);      // 25 Type of alarm
MyMessage VAR3Msg         (CHILD_ID,V_VAR3);      // 26 Text of Type
MyMessage VAR4Msg         (CHILD_ID,V_VAR4);      // 27 Text of Zone
MyMessage TEXTMsg         (CHILD_ID,V_TEXT);      // 47 Status messages

/* ************************************************************************************** */
#define DisplayID         0x37              // this is the ID in hex of the control panel Display ID we will uses (0x30 --> 0x37)
#define ControlPanelID    0x10              // address in hex of the D6100

static char MyBuffer [30];                  // a small working buffers
static char inString[30];
static char outString[30];
static char SendBuffer[30];

static char ApexBuffer [120] = "";          // a buffer to hold incoming serial message data from Apex D6100
static char msgBuffer [120] = "";           // a buffer to hold the complete incoming message from Apex D6100
static int pos = 0;                         // Current position of serial message data in the ApexBuffer

static char msgData[120] = "";              // Message data
static int  msgCS = 0;                      // Checksum of incomming message
static int  msgLength = 0;                  // Length of incomming message
static char msg[3] = "";                    // Message type

boolean stringComplete = false;             // A flag to tell us if an D6100 string is complete
static int newCS = 0;                       // New CS
static int msgType = 0;                     // Message type from NQ message
static int msgZone = 0;                     // Zone number from NQ message

unsigned long currentTime = 0;              
unsigned long lastSend = 0;

unsigned int temp = 0;

unsigned char SetTimeCtr = 0;               // Count of delay to send time to alarm 

#define AlarmSendDelay            250       // delay on each send to alarm panel
#define SendDelay                 250       // this is the delay after each send to MySensor
#define AckFlag                   false     // if we are requesting an ACK from GW

/* **************************************************************************** */
/* This is the System Notification Report Messages table, used only to dispaly message on debug port */

static const char *types[] =  {"Exterior Instant", "Exterior Delay 1", "Exterior Delay 2", "Interior Instant", "Interior Delay 1", "Interior Delay 2",
              "Fire", "Panic", "Silent", "Emergency", "Follower", "Aux", "Duress", "Duress not Armed", "Zone Restore after Activation",
              "Transmiter Low Battery", "Transmiter Low Battery Restore", "Zone Trouble", "Zone Trouble Restore", "Fuse Trouble", 
              "Fuse Trouble Restore", "Phone Line Restore", "Alarm Disarm", "Alarm Disarm after Activation", "Alarm Arm", "Alarm Arm after Activation",
              "Control Low Battery", "Control Low Battery Restore", "AC Power Fail", "AC Power Restore", "User Communication Test", "Auto Communication Test",
              "Cancle Alert", "Zone Bypass", "Zone Un-bypass", "Day Zone Trouble", "Day Zone Trouble Restore", "Up-Down Attempt", "Program Mode Entered",
              "Fall to Disarm", "Fail to Arm", "HWB416 Trouble", "HWB416 Trouble Restore", "Zone Open", "Zone Restore", "Zone Tamper", 
              "Zone Tamper Restore", "Radio Fail", "Radio Restore" };

static const char *zone[] = {"0 Zero", "1 Main Garage Interior Back Door", "2 Master Bedroom Office North Door", "3 Master Bedroom South Door",
                             "4 Family Room North Door", "5 Family Room South-East Door", "6 Family Room South-West Door", "7 Front Door",
                             "8 Entry Foyer North Door", "9 Entry Foyer North-Center Door", "10 Entry Foyer South-Center Door",
                             "11 Entry Foyer South Door", "12 Guest Bedroom East Door", "13 Guest Bedroom West Door", "14 Basement Door",
                             "15 Guest Office Door", "16 New Garage, Side South Door", "17 New Garage, Side North Door", "18 Main Garage North Door",
                             "19 Main Garage South Door", "20 New Garage, South Door's", "21 New Garage, North Door", "22 New Garage Interior Motion",
                             "23 FIRE Pool Equipment Room", "24 FIRE Patio North", "25 FIRE Main Garage South", "26 FIRE Main Garage North",
                             "27 FIRE Kitchen", "28 Smoke Kid's Wing", "29 Smoke Master Bedroom", "30 Smoke Guest Bedroom East",
                             "31 Smoke Guest Bedroom West", "32 Smoke Guest Office North", "33 Smoke Guest Office South", "34 Smoke Basement",
                             "35 Smoke Basement Theater", "36 FIRE Basement Utility", "37 FIRE New Garage", "38 FIRE New Garage Attic",
                             "39 FIRE New Garage Sprinkler Active", "40 FIRE Guest Wing Sprinkler Active", "41 4 Button Remote", "42 FIRE Studio",
                             "43 Studio Interior Motion", "44 Bedroom East Interior Motion", "45 Bedroom West Interior Motion", "46 Kids Wing Interior Motion",
                             "47 Family Room Interior Motion", "48 Main Garage Interior Motion", "49 Master Bedroom Interior Motion",
                             "50 Laundry Room Interior Motion", "51 FIRE Patio West", "52 Entry Foyer Motion", "53 LivingRoom Hallway Motion",
                             "54 Chicken Coupe Door Open", "55 FIRE Living Room", "56 Carbon Monoxide Kid's Wing", "57 Carbon Monoxide Master Bedroom",
                             "58 Carbon Monoxide Guest Wing Office", "59 Carbon Monoxide Basement", "60 Front Driveway Alarm", "61 Rear Driveway Alarm",
                             "62 Zone", "63 Zone", "64 Zone", "65 Zone", "66 Zone", "67 Zone", "68 Zone", "69 Zone", 
                             "70 Zone", "71 Zone", "72 Zone", "73 Zone", "74 Zone", "75 Zone", "76 Zone", "77 Zone", "78 Zone", "79 Zone",
                             "80 Zone", "81 Zone", "82 Zone", "83 Zone", "84 Zone", "85 Zone", "86 Zone", "87 Zone", "88 Zone", "89 Zone",
                             "90 Zone", "91 Zone", "92 Zone", "93 Zone", 
                             "94 Local Phone", "95 Phone Line Monitor", "96 Keypad", "97 Zone", "98 Zone", "99 System", "100 Zone"                           
                            };


/* *************************** Forward Declaration ************************************* */
void printDigits(int digits);
void digitalClockDisplay();
void Apex_Command (char *inString, char *outString);
void serialEvent1();
int  checkCS (char *myString);
int  parseMsg (char *msgBuffer);
void receive(const MyMessage &message);
void send2KeysD6100 (int temp);
void send1KeyD6100 (int temp);
void setTimeD6100 ();
time_t GetTime();
void SendTime();

/* ************************************************************************************** */
                            

/* **************************************************************************** */
/* ************************** Setup ******************************************* */
/* **************************************************************************** */
void setup()
{
  Serial1.begin (1200);                  // Apex D6100 panel

#ifdef __AVR_ATmega1284P__              // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
  debug(PSTR(" ** Hello from the Alarm System on a MoteinoMega **\n") );
  #else                                  // we must have a Moteino
  debug(PSTR(" ** Hello from the Alarm System on a Moteino **\n") );
#endif

  const char compile_file[]  = __FILE__ ;
  debug1(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug1(PSTR(" %s \n"), compile_file);
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug1(PSTR(" %s \n\n"), compile_date);
  debug1(PSTR(" Node ID: %u\n\n"), MY_NODE_ID);

  debug1(PSTR("*** Setting time from compile time\n"));
  setDateTime(__DATE__, __TIME__);        // set clock to compile time

//  Serial.print("Startup ");
//  digitalClockDisplay();

    wait (10000);                         // wait for alarm to be ready on power up
    
/* lets say a few words on the alarm display at startup time to be nice */
//  wait (1000);
//  Serial1.println ("0Bsi04900B5");      // Control
//  wait (500);
//  Serial1.println ("0Bsi09100B8");      // Is
//  wait (500);
//  Serial1.println ("0Bsi27800B1");      // Active
//  wait (500);

  Serial1.println ("08as0064");         // request Alarm Status to clear buffer
  wait (1000);

 // pinMode(LED1, OUTPUT);                 // Led

// Lets request time for clock and set alarms
  debug1(PSTR("*** Requesting time in Startup\n"));
  requestTime();                        // Request time from controller on startup
  setSyncInterval(3600);                // in sec,  once a day = 86400 sec, per hr = 3600
  setSyncProvider(GetTime);             // set function to call when time sync is required 
  wait (3000);

//  Serial.print("Startup ");
//  digitalClockDisplay();

  SetTimeCtr = 6;                       // this will force time set to D6100 on startup
  SendTime();                           // Send time to alarm if we have it

//  Alarm.alarmRepeat(8,30,0,SendTime);                   // 8:30am every day
//  Alarm.alarmRepeat(17,45,0,SendTime);                  // 5:45pm every day 
//  Alarm.timerRepeat(60, SendTime);                      // setup for test
  Alarm.alarmRepeat(dowSunday, 16, 30, 00,SendTime);      // 16:30:00 every Sunday set alarm
  
} // end of setup


/* ******************************************************************* */
// The time will be sent to the alarm if available
void SendTime()
{
  SetTimeCtr++;
  if ( SetTimeCtr >= 4)                     // we only want to send once every 4 weeks on sunday
  {
    SetTimeCtr = 0;
    if (year() >= 2017)                     // only set time if we have a valid time
    {
      debug1(PSTR("*** Sending time to Alarm\n"));
      setTimeD6100();                       // Set D6100 clock via keyboard
    }
  }
}


/* ******************************************************************* */
time_t GetTime()
{
  requestTime();                         // Request time from controller
}

/* ******************************************************************* */
/*  Print the time from Arduino internal clock */
void digitalClockDisplay()
{

// The first debug1 statement below is broken, if we have two or more %s in the same line, the first one is repeted
//Serial.print (dayStr(day()) );
//Serial.println (monthStr(month()));
//debug1(PSTR("Time: %d:%d:%02d %s %s %d\n"), hour(), minute(), second(), dayStr(day()), monthStr(month()), year() ); // <-- broken 
 
  debug(PSTR("*** Time: %d:%d:%02d %d/%d/%d\n"), hour(), minute(), second(), day(), month(), year() );

//  Serial.print("Time: ");
//  Serial.print(hour());
//  Serial.print(":");
//  Serial.print(minute());
//  Serial.print(":");
//  Serial.print(second());
//  Serial.print(" ");
//  Serial.print(day());
//  Serial.print("/");
//  Serial.print(month());
//  Serial.print("/");
//  Serial.println(year());
  
}


/* ******************************************************************* */
void presentation()  
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCHNAME, SKETCHVERSION, AckFlag);     wait (SendDelay);

  // Register this device as Custom sensor
  present(CHILD_ID, S_CUSTOM, "Alarm", AckFlag);  wait (SendDelay);             // S_CUSTOM = 23
}

/* **************************************************************************** */
/* This builds a proper command for the Apex D6100 by adding the length, extra 00 and the Checksum to the message */
void Apex_Command (char *inString, char *outString)
{
  char SendBuffer[40];
  sprintf (outString, "%02X", strlen (inString) + 6);  // get length of string
  strcat  (outString, inString);                       // insert length at beginning of outString
  strcat  (outString, "00");                            // add "00" to end of string
  int checksum = checkCS(outString);                    // compute CS
  sprintf (SendBuffer, "%02X", checksum);               // add it as a string
  strcat  (outString, SendBuffer);                      // add it to outBuffer
  Serial1.println (outString);  wait (AlarmSendDelay);  // send command to D6100
  debug1(PSTR("*** Cmd: %s\n"), outString);
}

/* **************************************************************************** */
void send2KeysD6100 (int temp)
{
        sprintf (SendBuffer, "zk%02X%02X00%02d%02d", ControlPanelID, DisplayID, temp/10, temp/10);
        strcpy(inString, SendBuffer );         
        Apex_Command (inString, outString);           // send first key
        
        sprintf (SendBuffer, "zk%02X%02X00%02d%02d", ControlPanelID, DisplayID, temp % 10, temp % 10);
        strcpy(inString, SendBuffer ); 
        Apex_Command (inString, outString);           // send 2nd key     
}

/* **************************************************************************** */
void send1KeyD6100 (int temp)
{       
        sprintf (SendBuffer, "zk%02X%02X00%02d%02d",ControlPanelID, DisplayID, temp % 10, temp % 10);
        strcpy(inString, SendBuffer );            
        Apex_Command (inString, outString);           // send first key
}


/* **************************************************************************** */
/*
  SerialEvent occurs whenever new data comes in the
  hardware serial RX port.  This routine is run between each
  time loop() runs, so using any delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent1()
{
  while ( Serial1.available() )
  {
    char inChar = (char) Serial1.read();      // get the next char from Apex D6100
    if (inChar > 0)                           // will be printable ASCII
    {
      switch (inChar)
      {
        case '\n':                            // Ignore new-lines
          break;

        case '\r':                            // Ignore CR and Return with a message complete
          stringComplete = true;              // Set complete flag on CR
          strncpy (msgBuffer, ApexBuffer, strlen(ApexBuffer));     // move to a working buffer
          msgBuffer [pos] = 0;                // make sure we have a null at end of message
          pos = 0;                            // clear the serial buffer pointer for next message
          break;

        default:
          ApexBuffer[pos++] = inChar;
          ApexBuffer[pos] = 0;                // add null to end of buffer

      } // end of switch
    }  // end of if
  } // end while
}

/* **************************************************************************** */
/* This will compute modulo 256 checksum of a buffer */
int checkCS (char *myString)
{
  int i = 0;
  unsigned char checksum = 0;
  while (myString [i])                  // will stop on Null
  {
    checksum += myString [i++];
  }
  return checksum = ~(checksum) + 1;    // twos complement
}


/* **************************************************************************** */
/* This will parse the incomming message from the alarm and will validate the checksum */
int parseMsg (char *msgBuffer)
{
  // lets parse the message data and then check the checksum...

  MyBuffer [0] = msgBuffer [0];
  MyBuffer [1] = msgBuffer [1];
  msgLength = strtol (MyBuffer, NULL, 16) - 4;      // this is the total length of the message
  debug2(PSTR("*** Msg Length: %d\n"), msgLength);

  msg [0] = msgBuffer [2];                        // lets get the message type
  msg [1] = msgBuffer [3];
  msg [2] = 0;

  debug2(PSTR("*** Msg: %s\n"), msg);

  MyBuffer [0] = msgBuffer [strlen(msgBuffer) - 2]; // let get the CS from the message
  MyBuffer [1] = msgBuffer [strlen(msgBuffer) - 1];
  MyBuffer [2] = 0;
  msgCS = strtol (MyBuffer, NULL, 16);              // this is the CS from message
  
  debug2(PSTR("*** Msg CS: %02X\n"), msgCS);
  
  msgBuffer[strlen(msgBuffer) - 2] = 0;           // remove the CS, we now have a copy of it

  newCS = checkCS(msgBuffer);                     // compute new checksum of message, without the original CS
  debug2(PSTR("*** New CS: %02X\n"), newCS);

  msgBuffer[strlen(msgBuffer) - 2] = 0;           // now remove the two 00 at end of message

  if (newCS == msgCS)                             // check to see if we have a valid CS
  {
    int i = 4;                                    // Need to get data, start at 1st message data byte, 4 bytes in from the start
    while ( msgBuffer [i])                        // will stop on Null
    {
      msgData[i - 4] = msgBuffer [i++];
    }
    
    msgData[i - 4] = 0;                           // null terminate the string
    debug2(PSTR("*** Data: %s\n"), msgData);
    debug2(PSTR("*** Data Length: %d\n"), strlen(msgData));
    return 0;
  }
  
  else
  {
    debug1(PSTR("*** Bad Checksum in Message\n"));
    return -1;
  }
}

/****************** Message Receive Loop ***************************
 * 
 * This is the message receive loop, here we look for messages address to us 
 * 
 * ***************************************************************** */

void receive(const MyMessage &message) 
{
   debug1(PSTR("*** Received message from gw"));  

  if (message.sensor == CHILD_ID )
  {
    if  (message.type==V_VAR1) 
      {
        debug1(PSTR("*** Received V_VAR1 message: %lu\n"), message.getULong());
      }
    
     if ( message.type==V_VAR2) 
      {
        debug1(PSTR("*** Received V_VAR2 message: %lu\n"), message.getULong());
      }
    
     if ( message.type==V_VAR3) 
      {
        debug1(PSTR("*** Received V_VAR3 message: %lu\n"), message.getULong());
      }

     if ( message.type==V_VAR3) 
      {
        debug1(PSTR("*** Received V_VAR4 message: %lu\n"), message.getULong());
      }
   }
}

/* **************************************************************************** */
/* receive the time from the gateway */
void receiveTime(unsigned long ts)
{
  debug1(PSTR("*** Received Time from gw: %lu \n"), ts);
  setTime(ts);                                            // Set from UNIX timestamp
//  Serial.print("RX ");
//  digitalClockDisplay();
}


/* **************************************************************************** */
/* This function will emulate the keyboard and send the keystrokes needed to set the time in the D6100 
* Its not a very elegant way to set the time, but it works... Time is from the Arduno time function
*/
void setTimeD6100 ()
{
        Serial1.println ("12zk10370011110069");     wait (AlarmSendDelay);   // Send time setup command to D6100 "8-2 keys"
        
/* We need to send 4 digit time is 12 hr format HHMM */
        temp = hour ();
        if ( temp > 12) temp = temp - 12;    
        send2KeysD6100( temp );
        send2KeysD6100( minute() );
//        debug1(PSTR("*** h:m %u:%02u\n"), temp, minute());
/* We need to send 1 = AM, 2 = PM */
        if ( isAM() )     send1KeyD6100 (1);           // Returns true if time now is AM 
        else              send1KeyD6100 (2);
       
/* We need to send the day of the week: 1 = Sunday, then month, day, 2 digit year */
        send1KeyD6100 ( weekday () );
        send2KeysD6100( month() );
        send2KeysD6100( day() );
        send2KeysD6100( year() - 2000 );
//       debug1(PSTR("*** Date: %u %u %u %un\n"), weekday (), month(), day(), year() - 2000);
}


/* **************************************************************************** */
/* ************************* LOOP ********************************************* */
/* **************************************************************************** */
void loop()
{
  _process ();
  Alarm.delay(1);                               // required to sync alarm time
  
  currentTime = millis();                       // get the current time
  if (currentTime - lastSend > WatchDog_FREQUENCY)
  {
    lastSend = currentTime;
    
    Serial1.println ("08as0064");               // Lets ask for the Alarm Status message as a watchdowg
 
    /* here we are building test messages for the D6100 */

    // These were use in debug to have the D6100 alarm send us messages 
    // Serial1.println ("08as0064");            // Alarm Status
    // Serial1.println ("08zp004E");            // Zone Partition Report
    // Serial1.println ("08zs004B");            // Zone Status Report
    // Serial1.println ("12zk10370011110069");  // Set time

    // This is use to build test messages to the alarm system
    //  strcpy(inString, "ASDDDDDDDD" );
    //  Apex_Command (inString, outString);
    //  Serial.print   ("Command: ");
    //  Serial.println (outString);

  }

/*  Lets see if we have a complete message from alarm */
  if (stringComplete)
  {
    stringComplete = false;                      // yes, then clear the string complete flag
    debug1(PSTR("*** We have a data from D6100: %s\n"), msgBuffer);
 
    if (parseMsg (msgBuffer) == 0)               // check for a valid message
    {
      if ( strcmp (msg, "NQ") == 0)              // NQ = System Event Notification
      {
        debug1(PSTR("*** NQ = System Event Notification: %s\n"), msgData);

        MyBuffer [0] = msgData [0];                 // let get the System Notification Report Type
        MyBuffer [1] = msgData [1];
        MyBuffer [2] = 0;
        msgType = strtol (MyBuffer, NULL, 16);
                
        MyBuffer [0] = msgData [2];                 // let get the zone
        MyBuffer [1] = msgData [3];
        msgZone = (strtol (MyBuffer, NULL, 16) +1); // Zone are reported offset by 1

        debug1(PSTR("*** Msg Type: %d %s, Zone: %d\n"), msgType, types[msgType], msgZone);
        debug1(PSTR("*** Zone Location: %d %s\n"), msgType, zone[msgZone]);
        
      /* Here we send our MySensor messages to the gateway */
        send(VAR1Msg.set((int) msgType), AckFlag);  wait(SendDelay);     // Send Message Type to gateway
        send(VAR2Msg.set((int) msgZone), AckFlag);  wait(SendDelay);     // Send Zone to gateway

        if (msgType > sizeof(types)/2) msgType = sizeof(types)/2 -1;
        send(VAR3Msg.set(types[msgType]), AckFlag); wait(SendDelay);     // Send Zone type to gateway

        if (msgZone > sizeof(zone)/2 ) msgZone = sizeof(zone)/2 -1;
        send(VAR4Msg.set(zone[msgZone]), AckFlag);  wait(SendDelay);     // Send Zone text to gateway
      }

      else if ( strcmp (msg, "CS") == 0)          // CS = Control Channel Status
      {
        debug1(PSTR("*** CS = Control Channel Status: %s\n"), msgData);
      }

      else if ( strcmp (msg, "AS") == 0)           // AS = Alarm Status Report
      {
       debug1(PSTR("*** AS = Alarm Status Report: %s\n"), msgData);
 
        switch (msgData [0])                      // for now we are only reporting Partition 1
        {
          case  'A':
          send(TEXTMsg.set("Armed to Away"), AckFlag);  wait(SendDelay);     // Send to gateway
          break;

          case  'H':
          send(TEXTMsg.set("Armed to Home"), AckFlag);  wait(SendDelay);     // Send to gateway
          break;

          case  'D':
          send(TEXTMsg.set("Alarm is Disarmed"), AckFlag);  wait(SendDelay);  // Send to gateway
          break;   
        }
      }

      else if ( strcmp (msg, "ZS") == 0)           // ZS = Zone Status Report
      {
        debug1(PSTR("*** ZS = Zone Status Report: %s\n"), msgData);
      }

      else if ( strcmp (msg, "ZP") == 0)           // ZP = Zone Partition Report
      {
        debug1(PSTR("*** ZP = Zone Partition Report: %s\n"), msgData);
      }

      else if ( strcmp (msg, "NK") == 0)           // NK = Keystroke Notification
      {
        debug1(PSTR("*** NK = Keystroke Notification: %s\n"), msgData);
      }

      else if ( strcmp (msg, "LR") == 0)           // LR = Location Read
      {
        debug1(PSTR("*** NK = Keystroke Notification: %s\n"), msgData);
      }

      else
      {
        debug1(PSTR("*** Unknown Message: %s\n"), msgData);
      }

    } // if (parseMsg (msgBuffer) == 0) 

  } // if (stringComplete)
}


/* This will take a compile string and convert it to a date time format for clock */

uint8_t conv2d(const char* p)
{
    uint8_t v = 0;

    if ('0' <= *p && *p <= '9')
    {
        v = *p - '0';
    }

    return 10 * v + *++p - '0';
}

void setDateTime(const char* date, const char* time)
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    year = conv2d(date + 9);

    switch (date[0])
    {
        case 'J': month = date[1] == 'a' ? 1 : month = date[2] == 'n' ? 6 : 7; break;
        case 'F': month = 2; break;
        case 'A': month = date[2] == 'r' ? 4 : 8; break;
        case 'M': month = date[2] == 'r' ? 3 : 5; break;
        case 'S': month = 9; break;
        case 'O': month = 10; break;
        case 'N': month = 11; break;
        case 'D': month = 12; break;
    }

    day = conv2d(date + 4);
    hour = conv2d(time);
    minute = conv2d(time + 3);
    second = conv2d(time + 6);

    // setDateTime(year+2000, month, day, hour, minute, second);
    setTime(hour,minute,second,day, month, year);
}


/* ************** The End ****************** */


