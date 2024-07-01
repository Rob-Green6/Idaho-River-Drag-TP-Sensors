//  Code to drive River Drag project hardware
//  Revision a, 2024.7.1
//  Rob Green, for University of Idaho, CER group
//  Licensed as open-source
//  NOTE:  Battery voltage reads are customized per installation
//  NOTE:  Sensor equilibration is done in software per installation
//  NOTE:  Serial Print statements can be turned off in this version
//         using a transformation starting around line 30
//  Hardware list:  Xiao esp32s3, PCA 9546A multiplexer, DFR0998 RTC,
//                  MS5837-02 pressure sensor, 26Ga cabling, 
//                  cat5e patch cabling, 
//  Companion Wifi app: Serial Wifi Terminal app by Kai Morrich


#include <Wire.h>
#include <MS5837.h>
#include <LittleFS.h>
#include <DFRobot_SD3031.h>
#include <TimeLib.h>
#include <WiFi.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <esp_sleep.h>

// MS5837 pressure related defines
#define PCAADDR 0x70
#define DEV_I2C Wire
// Display geometry for CER Logo
#define u8g_logo_width    60
#define u8g_logo_height   60

//Display geometry for battery voltage
#define u8g_batt_width    51
#define u8g_batt_height   19

#define Debug 1
#if Debug == 1
 #define debug(x) Serial.print(x)
 #define debugln(x) Serial.println(x)
#else
 #define debug(x)
 #define debugln(x)
#endif

MS5837 PATM;
MS5837 PSRF;
MS5837 PH2O;
DFRobot_SD3031 rtc;
sTimeData_t sTime;

// Display board definition from the commercial library, find that source???
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ D8, /* data=*/ D10, /* cs=*/ D7, /* dc=*/ D9, /* reset=*/ U8X8_PIN_NONE);

void i2cSelect();
void GETMS5837Patm();
void GETMS5837Psrf();
void GETMS5837Ph2o();
void Data_Print();
void Store_Data();  // Function to write data to an internal file
void manual_Blink(int blinks);  // user feedback of manual data collection mode.
void auto_Blink();  // user feedback of auto data collection mode.
void Wifi_Setup();  // Function to set up and turn on WiFi, then connect to "Serial to Wifi", by Kai Morich
void Wifi_Communication();
void handleWifiCommand(char TCP);
void drawLogo();  // display a splash screen on 128 x 64 OLED
void displayOLED1();  // display data in area 1 during run mode on 128 x 64 OLED
void displayOLED2();  // display data in area 1 during run mode on 128 x 64 OLED
void rtc_Time();  // retrieve time as year/month/day, hour:minute:second from RTC
void arduinoTime(); // retrieved internally calculated time based on TimeLib.h
void print_arduinoTime(); // Prints arduino time as a check
void errorOLED1();    // called when a code or hardware error will prevent good data collection
void displayOLED_Count();  // displays the count down number in auto mode
void readClientInput();  // reads string inputs from the Wifi app to adjust features
void deepSleep();
void batteryVoltage();
void battery4Bar();
void battery3Bar();
void battery2Bar();
void battery1Bar();
void battery0Bar();
void batteryCharge();
void batteryOff();

int z = 0;  // used to repeat an attempt to begin the RTC chip
int shrt = 2;  // number of blinks
int auto_Delay = 10;  // delay for auto mode, can be set through Wifi app
int auto_Count = auto_Delay;  // used to count down the auto timer in auto mode
int mode_Select = 1;  //mode =1 is automatic, mode = 0 is manual
int old_Select = 5;  // used to see if the mode_Select has changed
int stake = 0;
int drag = 1;

int currentHour;
int currentMinute;
int currentSecond;
int currentDay;
int currentMonth;
int currentYear;
int oldSecond;

String TCP_String;
String auto_Input;
char userRead;
char Lineatm[] = "Atm.";
char Linesrf[] = "Srf.";
char Lineh2o[] = "H2o.";
bool button_Manual = HIGH;  // dummy value, used to read switch high / low
bool connected = false;  // test variable for Wifi connection
bool Wifi_Start = HIGH;  //  dummy value so Wifi does not start automatically
bool old_Wifi = LOW;  // used to see if Wifi_Start has changed
bool code_Error = false;  // set to true when some error prevents the program functioning
File file;
File dir;
float Mbaratm;
float Ptempatm;
float Mbarsrf;
float Ptempsrf;
float Mbarh2o;
float Ptemph2o;
float Vbattf;

// Set up WiFi to be used with the Serial to WiFi app
const char* ssid = "Idahostreams";  // Name of wifi acess point
const uint16_t portNumber = 80;     // port used for wifi access point
WiFiServer server(portNumber);
WiFiClient client;

void setup() 
{
  setCpuFrequencyMhz(80);
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(10000);
  delay(100);

  // Set LittleFS up for file system      
  if (!LittleFS.begin(true)) 
  {
    debugln("*** Failed to mount file system ***");
    errorOLED1();
    return;
  }

  // initialize built in LED digital pin as an output.
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, HIGH);

  // initialize the battery voltage measurement pin
  pinMode (A0, INPUT);

  // initialize D8 pin for external LED as output
  pinMode (D2, OUTPUT);
  digitalWrite (D2, LOW);

  // initialize momentary switch pin; high is normal, low is start WiFi
  pinMode (D3, INPUT_PULLUP);

  // initialize manual / auto record button
  pinMode (D6, INPUT_PULLUP);

  Serial.println("   ... Good Morning ...");
  Serial.println("... Welcome to Pressure ...");

  debugln("");
  drawLogo();  // Boot up splash screen
  delay(1000);
  batteryVoltage();
  rtc_Time();
  print_arduinoTime();
  displayOLED2();  // Update frame area 2 with current information
}

void loop() 
{
  // Read the "activate Wifi" pin.
  Wifi_Start = digitalRead(D3);
  delay (50);

  // Call appropriate functions to start and handle Wifi
  if (Wifi_Start == LOW)
  {
   displayOLED2();  // Update frame area 2 with current information
   Wifi_Setup();
   Wifi_Start = HIGH;
  }
   
  displayOLED2();  // Update frame area 2 with current information
  // Check to see if data collection is in manual or automatic?
  if (mode_Select == 0)  // manual mode
  {
   //temperaturesCapture();
   displayOLED1();  // Update frame area 1 with current deep h2o temperature
   button_Manual = digitalRead(D6);
   if (button_Manual == LOW)
    { 
      manual_Blink(shrt);
      // Call voids that opens each i2c line and check for correct presence of MS5837 sensors
      GETMS5837Patm();
      GETMS5837Psrf();
      GETMS5837Ph2o();
      Data_Print();
      Store_Data();
      displayOLED1();  //  Update frame area 1 with "Record" feedback
      button_Manual = HIGH;
    }
    else
    {
      delay(1900);
    }
  }
 if (mode_Select == 1)  // auto mode
  {
    auto_Blink();
    GETMS5837Patm();
    GETMS5837Psrf();
    GETMS5837Ph2o();
    Data_Print();
    Store_Data();
    displayOLED1();  //  Update frame area 1 with "Record" feedback
    auto_Count = auto_Delay;
  }
}

void manual_Blink(int blinks) 
{ // *** FUNCTION TO BLINK THE BUILT IN LED, INDICATING THE PROGRAM IS RUNNING ***
  digitalWrite (LED_BUILTIN, LOW);
  digitalWrite (D2, HIGH);
  delay(50);
  digitalWrite (LED_BUILTIN, HIGH);
  digitalWrite (D2, LOW);
}

void auto_Blink() 
{ // *** FUNCTION TO BLINK THE BUILT IN LED, INDICATING THE PROGRAM IS RUNNING ***
 auto_Count = auto_Delay;
 oldSecond = 1000;
 displayOLED1();  //  Update frame area 1 with H2O Temperature
 while (auto_Count > 0)
 {
   if (second() != oldSecond) 
   {
    button_Manual = digitalRead(D6);  // first check if the manual record button is pressed
    if (button_Manual == LOW)
    {
      button_Manual = HIGH;
      break;
    }
    else   // proceed with normal auto mode if D6 is not low
    {
     displayOLED_Count();
     digitalWrite (LED_BUILTIN, LOW);
     digitalWrite (D2, HIGH);
     delay(50);
     digitalWrite (LED_BUILTIN, HIGH);
     digitalWrite (D2, LOW);
     auto_Count = auto_Count - 1;
     oldSecond = second();
    }
   }
 }
}

void GETMS5837Patm() 
{ 
 // *** FUNCTION TO COLLECT PRESSURE DATA FROM THE ATMOSPHERIC, (TOP), SENSOR ***
 i2cSelect(1);
 // Set up pressure sensor on line 1 from multiplexer.
 int error = 5;    
 if (!PATM.init()) 
 {
   delay(10);
   debugln("*** Sensor for Atm Init Failed ***");
 }
 PATM.setModel (MS5837::MS5837_02BA);
 PATM.setFluidDensity(997);
 //  check for correct presense of sensor on line 1 from multiplexer.
 Wire.beginTransmission(0X76);
 error = Wire.endTransmission();
 if (error == 0)
  { // This branch reads the sensor on line 1, 
    //debugln("...Found MS5837 on I2C for Atm...");
    PATM.read();
    Mbaratm = PATM.pressure(); + 1;
    Ptempatm = PATM.temperature(); + 1;
    delay (10);
  }
    else
  {
    debugln("*** No MS5837 on I2C for ATM ***");
    Mbaratm = 0.0;
    Ptempatm = 1000.0;
  }
}

void GETMS5837Psrf() 
{ 
 // *** FUNCTION TO COLLECT PRESSURE DATA FROM THE WATER SURFACE SENSOR ***
 i2cSelect(2);
 // Set up pressure sensor on line 1 from multiplexer.
 int error = 5;    
 if (!PSRF.init()) 
 {
   delay(10);
   debugln("*** Sensor for Srf Init Failed ***");
 }
 PSRF.setModel (MS5837::MS5837_02BA);
 PSRF.setFluidDensity(997);
 //  check for correct presense of sensor on line 2 from multiplexer.
 Wire.beginTransmission(0X76);
 error = Wire.endTransmission();
 if (error == 0)
  { // This branch reads the sensor on line 1, 
    //debugln("...Found MS5837 on I2C for SRF...");
    PSRF.read();
    Mbarsrf = PSRF.pressure(); + 1;
    Ptempsrf = PSRF.temperature(); + 1;
    delay (10);
  }
    else
  {
    debugln("*** No MS5837 on I2C for Srf ***");
    Mbarsrf = 0.0;
    Ptempsrf = 1000.0;
  }
}

void GETMS5837Ph2o()
{
 // *** FUNCTION TO COLLECT PRESSURE DATA FROM THE SUBMERGED, (BOTTOM), SENSOR ***
 i2cSelect(3);
 // Set up pressure sensor on line 3 from multiplexer.
 int error = 5;    
 if (!PH2O.init()) 
 {
   delay(10);
   debugln("*** Sensor for H2O Init Failed ***");
 }
 PH2O.setModel (MS5837::MS5837_02BA);
 PH2O.setFluidDensity(997);
 //  check for correct presense of sensor on line 3 from multiplexer.
 Wire.beginTransmission(0X76);
 delay(1000);
 error = Wire.endTransmission();
 if (error == 0)
  { // This branch reads the sensor on line 3, then calls the print void
    //debugln("...Found MS5837 on I2C for H2O...");
    //debugln("");
    PH2O.read();
    Mbarh2o = PH2O.pressure(); + 1;
    Ptemph2o = PH2O.temperature(); + 1;
    delay (10);

  }
    else
  {
    debugln("*** No MS5837 on I2C for H2O ***");
    Mbarh2o = 0.0;
    Ptemph2o = 1000.0;
  }
}

void Data_Print() 
{ //*** FUNCTION TO PRINT TEMP DATA TO A SCREEN ***
 // Pressure related data
  debug("MS5837 ");
  debug(Lineatm);
  debug("Pressure is ");
  debug(Mbaratm);
  debugln(" mbar.");

  debug("MS5837 ");
  debug(Lineatm);
  debug(" Temp. is ");
  debug(Ptempatm);
  debugln(" deg C.");
  debugln("");

  debug("MS5837 ");
  debug(Linesrf);
  debug("Pressure is ");
  debug(Mbarsrf);
  debugln(" mbar.");

  debug("MS5837 ");
  debug(Linesrf);
  debug(" Temp. is ");
  debug(Ptempsrf);
  debugln(" deg C.");
  debugln("");

  debug("MS5837 ");
  debug(Lineh2o);
  debug("Pressure is ");
  debug(Mbarh2o);
  debugln(" mbar.");

  debug("MS5837 ");
  debug(Lineh2o);
  debug(" Temp. is ");
  debug(Ptemph2o);
  debugln(" deg C.");
  debugln("");
  debugln("");  
}

void Store_Data()
{
  // *** FUNCTION FOR WRITING OR APPENDING DATA TO DatatempC.txt ***
  if (!LittleFS.begin()) 
  {
    client.println("*** Failed to mount LittleFS ***");
  }
  File file = LittleFS.open("/datatempC.txt", "a");
  if (!file) 
  {
    debugln("*** Failed to create or open a file ***");
    errorOLED1();
    return;
  }
    
  //sTime = rtc.getRTCTime();
  // Send date & time infoc comma dlimited, to the data file
  arduinoTime();
  file.print(currentYear);
  file.print("/");
  file.print(currentMonth);
  file.print("/");
  file.print(currentDay);
  file.print(", ");
  file.print(currentHour);
  file.print(":");
  file.print(currentMinute);
  file.print(":");
  file.print(currentSecond);
  file.print(", ");

  // Send MS5837 pressure and temp data, comma delimited, to the data file
  file.print(Mbaratm);
  file.print(", ");
  file.print(Ptempatm);
  file.print(", ");

  file.print(Mbarsrf);
  file.print(", ");
  file.print(Ptempsrf);
  file.print(", ");

  file.print(Mbarh2o);
  file.print(", ");
  file.println(Ptemph2o);
  //file.print(", ");

  // close the file and wrap up
  delay(10);
  file.close();
  debugln("... Data written to file ...");
  debugln("       ... Done ...");
  debugln("");
}

void i2cSelect(uint8_t i) 
{ // *** FUNCTION TO SELECT AN I2C LINE, 0-3 ***
  if (i > 3) return;
 
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void Wifi_Setup()
{ // *** FUNCTION FOR WIFI SETUP ***
  debugln("... Wifi Setup Start ...");
  debug("... Creating AP (Access Point) with name # ");
  debug(ssid);
  debugln(" # ...");
  WiFi.softAP(ssid);
  // Set the transmit power, can be used to reduce or increase
  // The espressif datasheet references the folowing values
  //  +19.5dBm @ 240mA  +16dBm @ 190mA   &   +14dBm @ 180mA
  // The WiFi.h library actually has many more power settings.
  // Use a provided "enum" value, starting wtih WIFI_POWER_ ...
  WiFi.setTxPower(WIFI_POWER_11dBm);
  IPAddress IP = WiFi.softAPIP();
  debug("... softAP with IP address: ");
  debug(IP);
  debugln(" ...");
  server.begin();
  debug("... TCP-Server on port ");
  debug(portNumber);
  debugln(" started ...");

   // Wait 30 seconds while user connects to AP WiFi
   Serial.println("... Connect Client Now ...");
   for (int j=0; j<19; j++) 
   {
     digitalWrite (LED_BUILTIN, LOW);
     digitalWrite (D2, HIGH);
     delay(1000);
     digitalWrite (LED_BUILTIN, HIGH);
     digitalWrite (D2, LOW);
     delay(100);

     Wifi_Communication();
   }
}

void Wifi_Communication()
{
 //*** FUNCTION FOR COMMUNICATION CONDITION ***
  Serial.flush();
  char TCP_Char;

  if (!connected) 
  {
    // listen for incoming clients
    client = server.available();
    if (client) 
    {
      debug("\n New client connected to WiFi AP !\n");
      if (client.connected()) 
      {
        debug("  Client now connected via TCP !\n");
        connected = true;
      } else 
      {
        debugln("but client is not connected over TCP !");        
        client.stop();  // close the connection:
      }
    }
  } 

  while (connected)
  {
    if (client.connected()) 
    {
      // if character has been sent from the client and is in the buffer
      while (client.available()) 
      { 
        TCP_Char = client.read(); // take 1st character out of the TCP-receive-buffer
        Serial.write(TCP_Char);   // print it to the serial monitor

        // Call the handle function to execute commands associated with the received character
        handleWifiCommand(TCP_Char);
      }  
    } 
    else 
    {
      debugln("... Client has disconnected the TCP-connection ...");
      client.stop();  // close the connection:
      connected = false;
    }
 }
}

void handleWifiCommand(char TCP) 

{ //*** FUNCTION TO COMMUNICATE WITH ANDROID APP MENU
  // grouping relates to the 1st line of menu items
   switch (TCP) 
   {
    // DISPLAYS THE 1st LINE MENU
    case 'M':
    client.println("\nWelcome to Idahostreams Data Collection Menu");
    client.println("          ....   Button Commands   ....\n");
    client.println("Menu 1 - Displays the menu 1 options");
    client.println("Dir    - Displays files in data directory");
    client.println("Size   - Displays file size of tempdataC.txt");
    client.println("Read   - Displays datatempC.txt for saving");
    client.println("Del    - Permanently deletes datatempC.txt");
    client.println("Done   - WiFi communication is shut down\n");
    break;

    // DISPLAYS DIRECTORY LISTING OF ROOT DIRECTORY UPON RECEEIVING 'D'
    case 'D':
    client.println("\n... Directory contents are: ...");
    delay(100);
      if (!LittleFS.begin()) {
        client.println("Failed to mount LittleFS");
        break;
      }

    // Open the directory
    dir = LittleFS.open("/");

    // Iterate through each file in the directory
    while (true) {
     // Get the file name
     file = dir.openNextFile();
     if (!file) {
      // no more file
      return;
     }
     client.println(file.name());
     file.close();
    }
    dir.close();
    LittleFS.end();
    break;

    // DISPLAY FILE SIZE OF datatempC.txt AND ANY OTTHER FILE IN ROOT DIR.
    case 'S':
    client.println("\nFile sizes are:");
    delay(100);
      if (!LittleFS.begin()) {
        client.println("Failed to mount LittleFS");
        break;
      }

    // Open the directory
    dir = LittleFS.open("/");

    if (!dir.isDirectory()) {
      Serial.println("Not a directory");
      break;
    }

    // Iterate through each file in the directory
    while (true) {
      file = dir.openNextFile();
      if (!file) {
        // No more files
        return;
      }
    
    client.print(file.name());
    client.print(" ,  ");
    client.println(file.size());
    file.close();
    }
    dir.close();
    LittleFS.end();
    break;

   // DISPLAYS CONTENTS OF datatempC.txt LINE BY LINE UPON RECEIVING 'R'
   case 'R':
   client.println("\n... The file will be printed to screen now ...\n");
   delay(20);
   if (!LittleFS.begin()) 
   {
     client.println("... LittleFS mount failed ...");
     return;
   }

   //Open the file for reading
   file = LittleFS.open("/datatempC.txt", "r");
   if (!file) 
   {
     client.println("File open failed..");
     return;
   }

   // Read and print the file contents line by line
   while (file.available()) 
   {
     String line = file.readStringUntil('\n');
     client.println(line);
   }
   // close file
   file.close();
   break;

   // DELETE datatempC.txt FOR NEXT ROUND OF DATA COLLECTION UPON RECEIVING 'D'
   // Check and see if a file already exists
   // If it does delete it
   case 'X':
   if (LittleFS.exists("/datatempC.txt")) 
   {
     LittleFS.remove("/datatempC.txt");
     client.println("\n... Existing File deleted ...");
   } else 
   {
     client.println("\n... No prior file exists ...");
   }
   LittleFS.end();
   break;
  
   // SHUTDOWN AFTER DATA COLLECTION UPON RECEIVING 'Z'
   case 'Z':
   if (stake == 1) 
   {
     // Send confirming message of ending communication and enter sleep
     client.println("\n ... Entering deep sleep now ...");
     client.println("        ... good night ...");
     LittleFS.end();

     //*** Sends ESP32S3 into deep sleep  ***
     //This turn off function is only for communication
     Serial.flush();
     esp_deep_sleep_start();
     break;
    }
   if (drag == 1)
   {
     client.println("\n ... Going back to data collection now ...");
     client.println("                  ... Stay Dry ...");
     LittleFS.end();
     Serial.flush();
     client.stop();  // close the connection:
     break;
   }

    // grouping relates to the 2nd line of menu items
    // DISPLAYS THE 2nd LINE MENU
    case 'L':
     client.println("\nWelcome to Idahostreams Sensor Drag Menu");
     client.println("         ....   Button Commands   ....\n");
     client.println("Menu 2 - Displays the menu 2 options");
     client.println("Man    - Selects manual trigger for sensor drag mode");
     client.println("Auto   - Selects automatic trigger for sensor drag mode");
     client.println("T Dlta - Selects the time delta used in auto mode");
     client.println("Time   - Sets RTC date and time, stored permanently");
     client.println("Chrge  - Exits data collection and charges battery\n");
    break;

    // Selects manaual data collection method for River Drag
    case 'N':
     mode_Select = 0;
     client.println("\n   ... MANUAL mode has been selected ...");
    break;

    // Selects automatic data collection method for River Drag
    case 'A':
     mode_Select = 1;
     client.println("\n   ... AUTOMATIC mode has been selected ...");
    break;    

    case 'F':
     Serial.flush();
     client.println("\nEnter the RECORD cycle time in seconds");
     client.println("Enter an integer between 2-99 seconds");
     client.println("   followed by !, eg:  14!");
     userRead = 'F';
     delay(10000);
     readClientInput();
     client.print("\nThe new time between readings will be: ");
     client.print(auto_Delay);
     client.println(" seconds");
     userRead = 'w';
    break;

    case 'V':
     u8g2.clear();
     delay(100);
     batteryCharge();
     delay(700);
     u8g2.clear();
     delay(100);
     batteryCharge();
     delay(700);
     u8g2.clear();
     delay(100);
     batteryCharge();
     delay(4000);
     // Begin to power items down
     u8g2.setPowerSave(1);
     // Send confirming message of ending communication and enter sleep
     client.println("\n ... Entering deep sleep now ...");
     client.println("        ... good night ...");
     LittleFS.end();
     //*** Sends ESP32S3 into deep sleep  ***
     //This turn off function is only for communication
     Serial.flush();
     esp_deep_sleep_start();
    break;

    case 'Q':
    //rtc.setHourSystem(rtc.e24hours);    // Set display format, use only when seeting initial time with a new battery
    //rtc.setTime(2024, 4, 9, 10, 55, 0);  // Set intial time when using RTC for first time with new battery
    char answer = 'n';
    int setyear = 0;
    int setmonth = 0;
    int setdate = 0;  
    int sethours = 0;
    int setminutes = 0;
    int setseconds = 0;
    // receive a time setting string and pass this to the RTC
    client.println("\nLet's set the date & time into the RTC back up");
    delay(100);
    client.println("Enter a date and time about a minute in the future");
    delay(100);
    client.println("Format as AA:BB:CC:XX:YY:ZZ!, \nyear:month:date:hours:minutes:seconds!");
    delay(100);
    client.println("End the input with a !, then press send\n");
    userRead = 'Q';
    delay(20000);
    readClientInput();

    // Check the input received for correctness
    client.print("\nI received; ");
    client.println(auto_Input);
    delay(5000);

      rtc.setHourSystem(rtc.e24hours);    // Set display format, use only when seeting initial time with a new battery
      int colonIndex1 = auto_Input.indexOf(':'); // Find the first ":"
      int colonIndex2 = auto_Input.indexOf(':', colonIndex1 + 1); // Find the second ":"
      int colonIndex3 = auto_Input.indexOf(':', colonIndex2 + 1); // Find the third ":"
      int colonIndex4 = auto_Input.indexOf(':', colonIndex3 + 1); // Find the fourth ":"
      int colonIndex5 = auto_Input.indexOf(':', colonIndex4 + 1); // Find the fifth ":"

      setyear = auto_Input.substring(0, colonIndex1).toInt(); // Extract year as integer
      setmonth = auto_Input.substring(colonIndex1 + 1, colonIndex2).toInt(); // Extract month as integer
      setdate = auto_Input.substring(colonIndex2 + 1, colonIndex3).toInt(); // Extract date as integer
      sethours = auto_Input.substring(colonIndex3 + 1, colonIndex4).toInt(); // Extract hours as integer
      setminutes = auto_Input.substring(colonIndex4 + 1, colonIndex5).toInt(); // Extract minutes as integer
      setseconds = auto_Input.substring(colonIndex5 + 1).toInt(); // Extract seconds as integer

      rtc.setTime(setyear, setmonth, setdate, sethours, setminutes, setseconds);  // Set intial time when using RTC for first time with new battery
      client.println("Great, the date and time are successfully entered");

    userRead = 'w';
    break;
 }
}


void drawLogo()
{
  u8g2.begin();
  u8g2.setPowerSave(0);
  delay(10);
  // 'CER Logo Graphic', 60 x 60px
  static unsigned char u8g_logo_bits[] = 
  {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 
	0x7e, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x9e, 0x00, 0x02, 0x19, 0x00, 0x00, 0x00, 0x00, 
	0xba, 0x80, 0x8f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x32, 0xc1, 0xd8, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x62, 0x32, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0xe2, 0x0e, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0xc2, 0x0c, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x82, 0x1c, 0x38, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x18, 0x1c, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, 0x0e, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x70, 0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0xe0, 0x03, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0xe0, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0xe0, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x60, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x38, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x18, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x1c, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0c, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x0e, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x07, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x07, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x07, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfc, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 
	0xfe, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0xfe, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 
	0x82, 0x0f, 0x00, 0x60, 0x00, 0x00, 0x00, 0x04, 0x82, 0x1f, 0x00, 0x60, 0x60, 0x03, 0x00, 0x04, 
	0x82, 0x3f, 0x00, 0x60, 0x1e, 0xf8, 0x7f, 0x04, 0x82, 0x7f, 0x00, 0xf0, 0x03, 0x00, 0x40, 0x04, 
	0x82, 0x7f, 0x00, 0x38, 0x00, 0x00, 0x80, 0x04, 0x82, 0xff, 0x00, 0x08, 0x00, 0x00, 0x80, 0x04, 
	0x82, 0xff, 0x03, 0x08, 0x00, 0x00, 0x80, 0x04, 0x02, 0xef, 0x07, 0x10, 0x00, 0x00, 0x00, 0x07, 
	0x02, 0xcf, 0x0e, 0x60, 0x00, 0x00, 0x00, 0x06, 0x02, 0x9b, 0x3d, 0xe0, 0x00, 0x00, 0x00, 0x06, 
	0x02, 0x1a, 0x73, 0xe0, 0x03, 0x00, 0x00, 0x04, 0x02, 0x36, 0xe7, 0x61, 0x0e, 0x00, 0x00, 0x00, 
	0x02, 0x36, 0x8e, 0x63, 0x30, 0x3e, 0x00, 0x00, 0x02, 0x6c, 0x18, 0x7e, 0x20, 0xc1, 0x00, 0x00, 
	0x02, 0xcc, 0x30, 0x7c, 0xe0, 0x00, 0x0e, 0x00, 0x02, 0xdc, 0xe0, 0xf0, 0x01, 0x00, 0x08, 0x00, 
	0x02, 0x98, 0xc1, 0xe1, 0x07, 0x00, 0x08, 0x00, 0x02, 0x38, 0x83, 0xe3, 0x1f, 0x00, 0x08, 0x04, 
	0x02, 0x30, 0x06, 0x66, 0xf8, 0xe0, 0x07, 0x06, 0x02, 0x60, 0x0c, 0x7c, 0xe0, 0x1f, 0x00, 0x06, 
	0x02, 0xc0, 0x18, 0x70, 0x00, 0x0f, 0x00, 0x07, 0x02, 0x80, 0x31, 0xe0, 0x01, 0x08, 0x00, 0x05, 
	0x02, 0x80, 0xe3, 0xe0, 0x07, 0x30, 0x80, 0x04, 0x02, 0x00, 0xc7, 0x61, 0x1f, 0xe0, 0x80, 0x07, 
	0x02, 0x00, 0x0e, 0x63, 0x7c, 0x80, 0xc1, 0x07, 0x02, 0x00, 0x1c, 0x6e, 0xf0, 0x01, 0x22, 0x06, 
	0xfe, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0xf3, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  // draw the CER logo Graphic with additional text
  
  u8g2.drawXBM( 2, 2, u8g_logo_width, u8g_logo_height, u8g_logo_bits);
  u8g2.setFont(u8g2_font_mozart_nbp_tf);	// choose a suitable font
  u8g2.drawStr(40,15,"University of");	// write something to the internal memory
  u8g2.drawStr(60,26,"Idaho");	// write something to the internal memory
  u8g2.drawStr(90,45,"CER");	// write something to the internal memory
  u8g2.drawStr(86,56,"Group");	// write something to the internal memory
  u8g2.sendBuffer(); // Transfer internal memory to the display
  delay(2000);
  u8g2.clear();
    u8g2.clearBuffer();					// clear the internal memory buffer
    u8g2.drawXBM( 2, 2, u8g_logo_width, u8g_logo_height, u8g_logo_bits);
    u8g2.sendBuffer(); // Transfer internal memory to the display
  delay(50);
}

void displayOLED1()
{
  if ((mode_Select == 0 & button_Manual != LOW) || (mode_Select ==1 & auto_Count == auto_Delay))
  {
   u8g2.clearBuffer();					// clear the internal memory buffer
   u8g2.setFont(u8g2_font_tiny5_tr);	// choose a suitable font
   u8g2.setCursor(40, 22);
   u8g2.print("H2O :                             C");  // write something to the internal memory buffer
   u8g2.updateDisplayArea(40/8, 16/8, 88/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
   u8g2.clearBuffer();					// clear the internal memory buffer
   u8g2.setFont(u8g2_font_profont22_tr);	// choose a suitable font
   u8g2.setCursor(64, 22);
   u8g2.print(Ptemph2o, 1);
   u8g2.updateDisplayArea(64/8, 1/8, 48/8, 24/8);  // divide by 8 because input is in 8 pixel tiles
  }
  else if ((mode_Select == 0 & button_Manual == LOW) || (mode_Select == 1 & auto_Count == 0))
  {
    u8g2.clearBuffer();					// clear the internal memory buffer
    u8g2.updateDisplayArea(40/8, 1/8, 88/8, 24/8);  // divide by 8 because input is in 8 pixel tiles
    u8g2.setFont(u8g2_font_profont22_tr);	// choose a suitable font
    u8g2.setCursor(48, 18);
    u8g2.print("RECORD");
    u8g2.updateDisplayArea(48/8, 1/8, 72/8, 24/8);  // divide by 8 because input is in 8 pixel tiles
    delay(250);
    u8g2.clearBuffer();					// clear the internal memory buffer
    u8g2.updateDisplayArea(48/8, 1/8, 72/8, 24/8);  // divide by 8 because input is in 8 pixel tiles
  }
}

void displayOLED2()
{
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.setFont(u8g2_font_tiny5_tr);	// choose a suitable font
  
  if (old_Select != mode_Select)
  {
   if (mode_Select == 0)
   {
     u8g2.updateDisplayArea(64/8, 32/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     u8g2.setCursor(64, 38);
     u8g2.print("MODE: manual");
     u8g2.updateDisplayArea(64/8, 32/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     old_Select = mode_Select;
   }
   else if (mode_Select == 1)
   {
     u8g2.updateDisplayArea(64/8, 32/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     u8g2.setCursor(64, 38);
     u8g2.print("MODE: auto");
     u8g2.updateDisplayArea(64/8, 32/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     old_Select = mode_Select;
   }
  }

  if (old_Wifi != Wifi_Start) 
  {
   if (Wifi_Start == HIGH)
   {
     u8g2.updateDisplayArea(64/8, 40/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     u8g2.setCursor(64, 46);
     u8g2.print("MODE: Wifi - OFF");
     u8g2.updateDisplayArea(64/8, 40/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     old_Wifi = Wifi_Start;
   }
   else if (Wifi_Start == LOW)
   {
     u8g2.updateDisplayArea(64/8, 40/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     u8g2.setCursor(64, 46);
     u8g2.print("MODE: Wifi - ON");
     u8g2.updateDisplayArea(64/8, 40/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
     old_Wifi = Wifi_Start;
   }
  }

  // Use the current date and time
  arduinoTime();
  char dateString[11];
  char TimeString[6];
  char minuteString[3];
  char lastDate[6];
  char nowMinute[3];
  char lastMinute[3];
  sprintf(dateString, "%02d.%02d.%02d", currentYear, currentMonth, currentDay);
  sprintf(TimeString, "%02d:%02d", currentHour, currentMinute);
  sprintf(minuteString, "%02d", currentMinute);

 if (strcmp(minuteString, lastMinute) != 0)
 {
   // display the current time
   u8g2.updateDisplayArea(64/8, 48/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
   u8g2.setCursor(64, 54);
   u8g2.print("TIME :");
   u8g2.setCursor(88, 54);
   u8g2.print(TimeString);
   u8g2.updateDisplayArea(64/8, 48/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
   sprintf(lastMinute, "%s", minuteString);
 }

 if (strcmp(dateString, lastDate) !=0)
 {
   u8g2.updateDisplayArea(64/8, 56/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
   u8g2.setCursor(64, 61);
   u8g2.print("DATE:");
   u8g2.setCursor(88, 61);
   u8g2.print(dateString);
   u8g2.updateDisplayArea(64/8, 56/8, 64/8, 8/8);  // divide by 8 because input is in 8 pixel tiles
   sprintf(lastDate, "%s", dateString);
 }
}

void errorOLED1()
{
  while (1)
  {
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(40/8, 1/8, 88/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  delay(250);
  u8g2.setFont(u8g2_font_profont22_tr);	// choose a suitable font
  u8g2.setCursor(48, 18);
  if ((Vbattf <= 2.8) && (Vbattf > 1))
  {
    batteryCharge();
  }
  else 
  {
    u8g2.print("ERROR");
  }
  u8g2.updateDisplayArea(48/8, 1/8, 72/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  delay(750);
  }
}

void displayOLED_Count() 
{
  u8g2.clearBuffer();					// clear the internal memory buffer.
  u8g2.updateDisplayArea(32/8, 1/8, 16/8, 16/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.setFont(u8g2_font_pcsenior_8n);	// choose a suitable font
  u8g2.setCursor(32,6);
  u8g2.print(auto_Count);
  u8g2.updateDisplayArea(32/8, 1/8, 16/8, 16/8);  // divide by 8 because input is in 8 pixel tiles
}

void rtc_Time() 
{
  // Code to get current time from DFRobot and create data row header of date and time sending it to the data file
  i2cSelect(0);
  debugln("... Starting RTC chip ...");
  int error = 5;
  while ((error != 0) & (z <= 100))
  { 
   if ((z == 0) || (z == 10) || (z == 20) || (z == 30) || (z == 40) || (z == 50) || (z == 60) || (z == 70) || (z == 80) || (z == 90))
   {
    debug("... z = ");
    debug(z);
    debugln(" ...");
   }
    error = rtc.begin();
    delay(100);
    z = z + 1;
  }
  if (z == 100)
  {
    debugln("*** no contact with RTC ***");
    errorOLED1();
  }
  z = 0;
  error = 5;

  debugln("... Testing RTC chip ...");
  while ((error != 0) & (z <= 100)) 
  {
    Wire.beginTransmission(0X32);
    error = Wire.endTransmission();
    if ((z == 0) || (z == 10) || (z == 20) || (z == 30) || (z == 40) || (z == 50) || (z == 60) || (z == 70) || (z == 80) || (z == 90)) 
    {
     debug("... z = ");
     debug(z);
     debugln(" ...");
    }
    z = z + 1;
  }

  if (error != 0)
  {
    debugln("*** No RTC on I2C 0X32 ***");
  }

  sTime = rtc.getRTCTime();
  setTime(sTime.hour, sTime.minute, sTime.second, sTime.day, sTime.month, sTime.year);
  arduinoTime();
}

void arduinoTime() 
{
  currentYear = year();
  currentMonth = month();
  currentDay = day();
  currentHour = hour();
  currentMinute = minute();
  currentSecond = second();
}

void print_arduinoTime()
{
  debug(currentYear);
  debug("/");
  debug(currentMonth);
  debug("/");
  debug(currentDay);
  debug(", ");
  debug(currentHour);
  debug(":");
  debug(currentMinute);
  debug(":");
  debugln(currentSecond);
  debugln("");
}

void readClientInput() 
{
  //*** FUNCTION FOR COMMUNICATION CONDITION ***
  auto_Input = "nothing";

  auto_Input = client.readStringUntil('!'); // take multiple characters out of the TCP-receive-buffer
  // Trim away the !
  auto_Input.trim();
  auto_Input.replace("!", "");

  if (userRead == 'F') 
  {
    // Convert the 2 digit auto Delay Timing to an integer
    String firstTwoDigits = auto_Input.substring(0, 2);
    auto_Delay = firstTwoDigits.toInt();
  }

  if (userRead == 'Q') 
  {

  }
}

void batteryVoltage()
{
  uint32_t Vbatt = 0;
  for(int k = 0; k<8; k++) 
  {
    Vbatt = Vbatt + analogReadMilliVolts(A0);
    delay(20);
  }
  //  
  Vbattf = 2 * Vbatt / 8 / 1000.0 * 1.8;
  debug("\nBattery Voltage is: ");
  debugln(Vbattf);
  debugln("");
  battery4Bar();
    delay(300);
  battery3Bar();
    delay(300);
  battery2Bar();
    delay(300);
  battery1Bar();
    delay(300);
  battery0Bar();
    delay(300);
  batteryCharge();
    delay(300);
  batteryOff();
    delay(300);

  if (Vbattf < 1.0) 
  {
    batteryOff();
  }

  if (Vbattf >= 3.5) 
  {
    battery4Bar();
  }
  else if (Vbattf >= 3.3) 
  {
    battery3Bar();
  }
  else if (Vbattf >= 3.1) 
  {
    battery2Bar();
  }
  else if (Vbattf >= 2.9) 
  {
    battery1Bar();
  }
  else if (Vbattf >= 2.8) 
  {
    battery0Bar();
  }
  else if ((Vbattf <= 2.8) && (Vbattf >= 1.0))
  {
    errorOLED1();
  }
  delay(4000);
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void battery4Bar() 
{
  // 'batt 4 bar', 51x19px
  static unsigned char u8g_batt_4_bar[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x02, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0x4f, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 
	0xf9, 0x4f, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0x4f, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 
	0x03, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 0x03, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 0x03, 0xf3, 
	0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 0x03, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 0x03, 0xf3, 0xcf, 0x7f, 
	0xfe, 0xf9, 0xcf, 0x03, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0xcf, 0x03, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 
	0x4f, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0x4f, 0x00, 0xf3, 0xcf, 0x7f, 0xfe, 0xf9, 0x4f, 0x00, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0xf8, 0xff, 
	0xff, 0xff, 0xff, 0x1f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_4_bar);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void battery3Bar() 
{
  // 'batt 3 bar', 51x19px
  static unsigned char u8g_batt_3_bar[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 
	0x03, 0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 
	0x03, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x07, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x07, 0xf3, 
	0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x07, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x07, 0xf3, 0x9f, 0x7f, 
	0xfe, 0x03, 0xc0, 0x07, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x03, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 
	0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x00, 0xf3, 0x9f, 0x7f, 0xfe, 0x03, 0xc0, 0x00, 
	0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xfc, 0xff, 
	0xff, 0xff, 0xff, 0x3f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_3_bar);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void battery2Bar() 
{
  // 'batt 2 bar', 51x19px
  static unsigned char u8g_batt_2_bar[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 
	0x00, 0xc0, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 
	0x03, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x07, 0xf3, 
	0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0xcf, 0x7f, 
	0x00, 0x00, 0xc0, 0x07, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x03, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 
	0xc0, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0xcf, 0x7f, 0x00, 0x00, 0xc0, 0x00, 
	0x03, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0xf8, 0xff, 
	0xff, 0xff, 0xff, 0x1f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_2_bar);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void battery1Bar() 
{
  // 'batt 1 bar', 51x19px
  static unsigned char u8g_batt_1_bar[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 
	0x00, 0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 
	0x03, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 
	0x0f, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x0f, 0x00, 
	0x00, 0x00, 0xc0, 0x07, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x03, 0xf3, 0x0f, 0x00, 0x00, 0x00, 
	0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0xe3, 0x0f, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xfc, 0xff, 
	0xff, 0xff, 0xff, 0x3f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_1_bar);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void battery0Bar() 
{
  // 'batt 0 bar', 51x19px
  static unsigned char u8g_batt_0_bar[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x33, 0x00, 0x00, 0x00, 
	0x00, 0xc0, 0x00, 0x73, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x73, 0x00, 0x00, 0x00, 0x00, 0xc0, 
	0x03, 0x73, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xf3, 0x01, 0x00, 
	0x00, 0x00, 0xc0, 0x07, 0xf3, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x03, 0xf3, 0x01, 0x00, 0x00, 0x00, 
	0xc0, 0x00, 0xf3, 0x03, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xf3, 0x03, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xfc, 0xff, 
	0xff, 0xff, 0xff, 0x3f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_0_bar);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void batteryCharge() 
{
  // 'batt Charge', 51x19px
  static unsigned char u8g_batt_Charge[] = 
 {
	0xfe, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x01, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x00, 0x01, 0x00, 0xc0, 0x01, 0x00, 0xc0, 0x00, 0x01, 0x00, 0xc0, 0x03, 
	0x00, 0xc0, 0x00, 0x01, 0x10, 0xc0, 0x0f, 0x00, 0xc0, 0x01, 0x01, 0xe0, 0xc1, 0x3f, 0x00, 0xc0, 
	0x07, 0x01, 0xc0, 0xdf, 0xff, 0x00, 0xc0, 0x07, 0x01, 0x00, 0xff, 0xff, 0x03, 0xc0, 0x07, 0x01, 
	0x00, 0xfc, 0xff, 0x07, 0xc0, 0x07, 0x01, 0x00, 0xf8, 0x0f, 0x1f, 0xc0, 0x07, 0x01, 0x00, 0xe0, 
	0x0f, 0x00, 0xc0, 0x07, 0x01, 0x00, 0x80, 0x0f, 0x00, 0xc0, 0x01, 0x01, 0x00, 0x00, 0x0e, 0x00, 
	0xc0, 0x00, 0x01, 0x00, 0x00, 0x0c, 0x00, 0xc0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00            
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_Charge);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}

void batteryOff() 
{
  // 'batt Off', 51x19px
  static unsigned char u8g_batt_Off[] = 
 {
	0xfc, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x03, 0x80, 0x0f, 0x0f, 0x07, 0x80, 
	0x07, 0x03, 0xc0, 0x18, 0x03, 0x01, 0x80, 0x0f, 0x03, 0x40, 0x18, 0x03, 0x01, 0x80, 0x0f, 0x03, 
	0x40, 0x18, 0x0f, 0x07, 0x80, 0x0f, 0x03, 0xc0, 0x18, 0x03, 0x01, 0x80, 0x0f, 0x03, 0x80, 0x0f, 
	0x03, 0x01, 0x80, 0x0f, 0x03, 0x00, 0x03, 0x00, 0x00, 0x80, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 
	0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 
	0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xfc, 0xff, 
	0xff, 0xff, 0xff, 0x7f, 0x00
 };
  u8g2.clearBuffer();					// clear the internal memory buffer
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
  u8g2.drawXBM( 64, 8, u8g_batt_width, u8g_batt_height, u8g_batt_Off);
  u8g2.updateDisplayArea(64/8, 1/8, 64/8, 32/8);  // divide by 8 because input is in 8 pixel tiles
}
