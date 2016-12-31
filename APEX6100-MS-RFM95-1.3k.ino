/*
 *  Ver 1.2k  20 Dec 2016 TRL 
 *  
 *  A program to control an APEX Destiny 6100(AN) Alarm panel. 
 *   This is running on a MoteinoMEGA, so we have lots of memory available
 *   Radio is 915MHz RFM95 LoRa
 *   Tested with MySensor 2.1.0 beta
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
 *  
 *  
 *
 *  Notes:  1)  Tested with Arduino 1.8.0
 *          2)  Testing using MoteinoMega LoRa Rev1 with RFM95
 *          3)  MySensor 2.1 30 Dec 2016
 *
 * Todo:
 *  get time from MQTT and set time in D6100 
 *  some cleanup of code
 *  fix TX LED issue
 *  
 */

/* ************************************************************************************** */
#include <string.h>
//#include <Time.h>                                 // this is the Arduino time functions
#include <TimeLib.h>
/* ************************************************************************************** */
// Most of these items below need to be prior to #include <MySensor.h> 

/*  Enable debug prints to serial monitor on port 0 */
//#define MY_DEBUG            // used by MySensor
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RFM95 
//#define MY_DEBUG1           // used in this program, level 1 debug
//#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME    "Alarm D6100"
#define SKETCHVERSION "1.3k RFM95"

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
unsigned long WatchDog_FREQUENCY = 60000;         // time to refresh gateway with alarm status ~1min

MyMessage VAR1Msg         (CHILD_ID,V_VAR1);      // 24
MyMessage VAR2Msg         (CHILD_ID,V_VAR2);      // 25
MyMessage VAR3Msg         (CHILD_ID,V_VAR3);      // 26
MyMessage VAR4Msg         (CHILD_ID,V_VAR4);      // 27
MyMessage TEXTMsg         (CHILD_ID,V_TEXT);      // 47
MyMessage CUSTOMMsg       (CHILD_ID,V_CUSTOM);    // 48

/* ************************************************************************************** */
#define DisplayID 0x37                      // this is the ID in hex of the control panel Display ID we will uses (0x30 --> 0x37)
#define ControlPanelID 0x10                 // D6100

static char inString [30] = "";             // used by serial command generator
static char outString[40] = "";
static char buffer [20];                    // a small working buffer

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

static const char *zone[] = {" 0 Null", " 1 Main Garage Interior Back Door", " 2 Master Bedroom Office North Door", " 3 Master Bedroom South Door",
                             " 4 Family Room North Door", " 5 Family Room South-East Door", " 6 Family Room South-West Door", " 7 Front Door",
                             " 8 Entry Foyer North Door", " 9 Entry Foyer North-Center Door", "10 Entry Foyer South-Center Door",
                             "11 Entry Foyer South Door", "12 Guest Bedroom East Door", "13 Guest Bedroom West Door", "14 Basement Door",
                             "15 Guest Office Door", "16 New Garage, Side South Door", "17 New Garage, Side North Door", "18 Main Garage North Door",
                             "19 Main Garage South Door", "20 New Garage, South Door's", "21 New Garage, North Door", "22 New Garage Interior Motion",
                             "23 Pool Equipment Room Fire", "24 Patio Fire, North", "25 Main Garage South Fire", "26 Main Garage North Fire",
                             "27 Kitchen Fire", "28 Kid's Wing Smoke", "29 Master Bedroom Smoke", "30 Guest Bedroom East Smoke",
                             "31 Guest Bedroom West Smoke", "32 Guest Office North Smoke", "33 Guest Office South Smoke", "34 Basement Smoke",
                             "35 Basement Theater Smoke", "36 Basement Utility Fire", "37 New Garage Fire", "38 New Garage Attic Fire",
                             "39 New Garage Sprinkler Active, Fire", "40 Guest Wing Sprinkler Active, Fire", "41 4 Button Remote", "42 Studio Fire",
                             "43 Studio Interior Motion", "44 Bedroom East Interior Motion", "45 Bedroom West Interior Motion", "46 Kids Wing Interior Motion",
                             "47 Family Room Interior Motion", "48 Main Garage Interior Motion", "49 Master Bedroom Interior Motion",
                             "50 Laundry Room Interior Motion", "51 Patio Fire, West", "52 Entry Foyer Motion", "53 LivingRoom Hallway Motion",
                             "54 Chicken Coupe Door Open", "55 LivingRoom Smoke-Fire", "56 Kid's Wing Carbon Monoxide", "57 Master Bedroom Carbon Monoxide",
                             "58 Guest Wing Office Carbon Monoxide", "59 Basement Carbon Monoxide", "60 Front Driveway Alarm", "61 Rear Driveway Alarm",
                             "62 Zone 62", "63 Zone 63", "64 Zone 64", "65 Zone 65", "66 Zone 66", "67 Zone 67", "68 Zone 68", "69 Zone 69"                          
                            };


/* *************************** Forward Declaration ************************************* */
void printDigits(int digits);
void digitalClockDisplay();
void Apex_Command (char *inString, char *outString);
void serialEvent1();
int checkCS (char *myString);
int parseMsg (char *msgBuffer);
void receive(const MyMessage &message);
void send2KeysD6100 (int temp);
void send1KeyD6100 (int temp);
void setTimeD6100 ();

/* ************************************************************************************** */
                            

/* **************************************************************************** */
/* ************************** Setup ******************************************* */
/* **************************************************************************** */
void setup()
{
  Serial.begin  (115200);                // Debug port
  Serial1.begin (1200);                  // Apex D6100 panel

 #ifdef __AVR_ATmega1284P__      // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
  debug(PSTR(" ** Hello from the Alarm System on a MoteinoMega **\n") );
  #else                         // we must have a Moteino
  debug(PSTR(" ** Hello from the Alarm System on a Moteino **\n") );
#endif


  const char compile_file[]  = __FILE__ ;
  debug(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug(PSTR(" %s \n"), compile_file);
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug(PSTR(" %s \n\n"), compile_date);
  debug(PSTR(" Network ID: %u  Node ID: %u\n\n"), MY_RFM69_NETWORKID, MY_NODE_ID);



/* lets say a few words on alarm display at startup to be nice */
  wait (1000);
  Serial1.println ("0Bsi04900B5");      // Control
  wait (500);
  Serial1.println ("0Bsi09100B8");      // Is
  wait (500);
  Serial1.println ("0Bsi27800B1");      // Active
  wait (500);

 // pinMode(LED1, OUTPUT);                 // Led


//  setTime(21,12,15,02,9,2017);          // Set Arduino clock to this time manually for testing
//  Serial.print("Time: ");
//  digitalClockDisplay();
//  setTimeD6100();                       // Set D6100 clock

//  debug1(PSTR("This is a test\n") );
//  debug2(PSTR("Msg Length: %d\n"), msgLength);

}

/* ******************************************************************* */
/* A utility function for testing the time function: prints preceding colon and leading 0 */
void printDigits(int digits)
{
  Serial.print(":");
  if(digits < 10) Serial.print('0');
  Serial.print(digits);
}

/* ******************************************************************* */
/*  Print the time from Arduino internal clock */
void digitalClockDisplay()
{
  Serial.print      (hour());
  printDigits       (minute());
  printDigits       (second());
  Serial.print      (" ");
  Serial.print      (day());
  Serial.print      (" ");
  Serial.print      (month());
  Serial.print      (" ");
  Serial.print      (year());
  Serial.println();
}


/* ******************************************************************* */
void presentation()  
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCHNAME, SKETCHVERSION);
  wait (250);
  
  // Register this device as Custom sensor
  present(CHILD_ID, S_CUSTOM);       // S_CUSTOM = 23
  wait (250);
}

/* **************************************************************************** */
/* This builds a proper command for the Apex D6100 by adding the length, extra 00 and the Checksum to the message */
void Apex_Command (char *inString, char *outString)
{
  sprintf (outString, "%02X", strlen (inString) + 6);   // get length of string
  strcat  (outString, inString);                        // insert length at beginning of outString
  strcat  (outString, "00");                            // add "00" to end of string
  int checksum = checkCS(outString);                    // compute CS
  sprintf (buffer, "%02X", checksum);                   // add it as a string
  strcat  (outString, buffer);                          // add it to outBuffer
  debug1(PSTR("Cmd: %s\n"), outString);
   
  Serial1.println (outString);                          // send command to D6100
}


/* **************************************************************************** */
/*
  SerialEvent occurs whenever new data comes in the
  hardware serial RX port.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
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
/* This will parse the incomming message and will validate the checksum */
int parseMsg (char *msgBuffer)
{
  // lets parse the message data and then check the checksum...

  buffer [0] = msgBuffer [0];
  buffer [1] = msgBuffer [1];
  msgLength = strtol (buffer, NULL, 16) - 4;      // this is the total length of the message
  debug1(PSTR("Msg Length: %d\n"), msgLength);

  msg [0] = msgBuffer [2];                        // lets get the message type
  msg [1] = msgBuffer [3];
  msg [2] = 0;

  debug1(PSTR("Msg: %s\n"), msg);

  buffer [0] = msgBuffer [strlen(msgBuffer) - 2]; // let get the CS from the message
  buffer [1] = msgBuffer [strlen(msgBuffer) - 1];
  buffer [2] = 0;
  msgCS = strtol (buffer, NULL, 16);              // this is the CS from message
  
  debug1(PSTR("Msg CS: %02X\n"), msgCS);
  
  msgBuffer[strlen(msgBuffer) - 2] = 0;           // remove the CS, we now have a copy of it

  newCS = checkCS(msgBuffer);                     // compute new checksum of message, without the original CS
  debug1(PSTR("New CS: %02X\n"), newCS);

  msgBuffer[strlen(msgBuffer) - 2] = 0;           // now remove the two 00 at end of message

  if (newCS == msgCS)                             // check to see if we have a valid CS
  {
    int i = 4;                                    // Need to get data, start at 1st message data byte, 4 bytes in from the start
    while ( msgBuffer [i])                        // will stop on Null
    {
      msgData[i - 4] = msgBuffer [i++];
    }
    
    msgData[i - 4] = 0;                           // null terminate the string
    debug1(PSTR("Data: %s\n"), msgData);
    debug1(PSTR("Data Length: %d\n"), strlen(msgData));
    return 0;
  }
  
  else
  {
    debug2(PSTR("Bad Checksum in Message\n"));
    return -1;
  }
}

/* **************************************************************************** */
/* This is the callback for any mesages that are address to us */
void receive(const MyMessage &message) 
{
   Serial.println("Received message from gw:");  

  if (message.sensor == CHILD_ID )
  {
    if  (message.type==V_VAR1) 
      {
        debug2(PSTR("Received V_VAR1 message from gw: %s\n"), message.getULong());
      }
    
     if ( message.type==V_VAR2) 
      {
        debug2(PSTR("Received V_VAR2 message from gw: %s\n"), message.getULong());
      }
    
     if ( message.type==V_VAR3) 
      {
        debug2(PSTR("Received V_VAR3 message from gw: %s\n"), message.getULong());
      }

     if ( message.type==V_VAR3) 
      {
        debug2(PSTR("Received V_VAR4 message from gw: %s\n"), message.getULong());
      }
   }
}

/* **************************************************************************** */
void send2KeysD6100 (int temp)
{
        sprintf (buffer, "zk%02X%02X00%02d%02d",ControlPanelID, DisplayID, temp/10, temp/10);
        strcpy(inString, buffer );            
        Apex_Command (inString, outString);   // send first key
        
        sprintf (buffer, "zk%02X%02X00%02d%02d",ControlPanelID, DisplayID, temp % 10, temp % 10);
        strcpy(inString, buffer ); 
        Apex_Command (inString, outString);   // send 2nd key     
}

/* **************************************************************************** */
void send1KeyD6100 (int temp)
{
        sprintf (buffer, "zk%02X%02X00%02d%02d",ControlPanelID, DisplayID, temp % 10, temp % 10);
        strcpy(inString, buffer );            
        Apex_Command (inString, outString);   // send first key
}

/* **************************************************************************** */
/* This function will emulate the keyboard and send the keystrokes needed to set the time in the D6100 
* Its not a very elegant way to set the time, but it works... Time is from the Arduno time function
*/
void setTimeD6100 ()
{
        Serial1.println ("12zk10370011110069");  // Send time mode 8-2 key to D6100
        
/* We need to send 4 digit time is 12 hr format HHMM */
        temp = hour ();
        if ( temp > 12) temp = temp - 12;
        send2KeysD6100( temp );
        send2KeysD6100( minute() );

/* We need to send 1 = AM, 2 = PM */
        if ( isAM() )     send1KeyD6100 (1);    // Returns true if time now is AM 
        else              send1KeyD6100 (2);
       
/* We need to send the day of the week: 1 = Sunday, then month, day, 2 digit year */
        send1KeyD6100 ( weekday () );
        send2KeysD6100( month() );
        send2KeysD6100( day() );
        send2KeysD6100( year() - 2000 );
}


/* **************************************************************************** */
/* **************************************************************************** */
/* **************************************************************************** */
void loop()
{

  _process ();
  
  currentTime = millis();                       // get the current time
  if (currentTime - lastSend > WatchDog_FREQUENCY)
  {
    lastSend = currentTime;

//#ifdef MY_RFM69_Enable_ATC
//    debug(PSTR("\n *** RSSI: %d  MY TX Pwr: %d\n\n"), _radio.getAckRSSI(), _radio._transmitLevel);
//#endif
    
    Serial1.println ("08as0064");             // Lets ask for the Alarm Status message as watchdowg
 
    // here we are building test messages for the D6100

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

/*  Lets see if we have a complete message */
  if (stringComplete)
  {
    stringComplete = false;                      // yes, then clear the string complete flag
    debug1(PSTR("We have a string: %s\n"), msgBuffer);
 
    if (parseMsg (msgBuffer) == 0)               // check for a valid message
    {
      if ( strcmp (msg, "NQ") == 0)              // NQ = System Event Notification
      {
        debug2(PSTR("NQ = System Event Notification: %s\n"), msgData);

        buffer [0] = msgData [0];                 // let get the System Notification Report Type
        buffer [1] = msgData [1];
        buffer [2] = 0;
        msgType = strtol (buffer, NULL, 16);
                
        buffer [0] = msgData [2];                 // let get the zone
        buffer [1] = msgData [3];
        msgZone = (strtol (buffer, NULL, 16) +1); // Zone are reported offset by 1

        debug2(PSTR("Msg Type: %d %s, Zone: %d\n"), msgType, types[msgType], msgZone);
        debug2(PSTR("Zone Location: %d %s\n"), msgType, zone[msgZone]);
        
      /* Here we send a MySensor messages */
        send(VAR1Msg.set((int) msgType, true));             // Send Message Type to gateway
        wait (200);
        send(VAR2Msg.set((int) msgZone, true));             // Send Zone to gateway
        wait (200);  
//        send(CUSTOMMsg.set(zone[msgZone], true));      // Send Zone text to gateway   

//CSTRING sStr((char*)buffer);
 
      }

      else if ( strcmp (msg, "CS") == 0)          // CS = Control Channel Status
      {
        debug2(PSTR("CS = Control Channel Status: %s\n"), msgData);
      }

      else if ( strcmp (msg, "AS") == 0)           // AS = Alarm Status Report
      {
       debug2(PSTR("AS = Alarm Status Report: %s\n"), msgData);
 
        switch (msgData [0])                      // for now we are only reporting Partition 1
        {
          case  'A':
          send(TEXTMsg.set("Armed to Away"));     // Send to gateway
          break;

          case  'H':
          send(TEXTMsg.set("Armed to Home"), true);     // Send to gateway
          break;

          case  'D':
          send(TEXTMsg.set("Alarm is Disarmed"), true); // Send to gateway
          break;   
        }
      }

      else if ( strcmp (msg, "ZS") == 0)           // ZS = Zone Status Report
      {
        debug2(PSTR("ZS = Zone Status Report: %s\n"), msgData);
      }

      else if ( strcmp (msg, "ZP") == 0)           // ZP = Zone Partition Report
      {
        debug2(PSTR("ZP = Zone Partition Report: %s\n"), msgData);
      }

      else if ( strcmp (msg, "NK") == 0)           // NK = Keystroke Notification
      {
        debug2(PSTR("NK = Keystroke Notification: %s\n"), msgData);
      }

      else if ( strcmp (msg, "LR") == 0)           // LR = Location Read
      {
        debug2(PSTR("NK = Keystroke Notification: %s\n"), msgData);
      }

      else
      {
        debug2(PSTR("Unknown Message: %s\n"), msgData);
      }

    } // if (parseMsg (msgBuffer) == 0) 

  } // if (stringComplete)
}

/* ************** The End ****************** */


