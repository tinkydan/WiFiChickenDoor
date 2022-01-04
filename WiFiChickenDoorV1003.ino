

#include <ADS1X15.h>

ADS1115 ADS(0x48);
/////////////////////////////////////////////////////////////////////////////////////////
///////////// LIBRARYS USED IN THE FOLLOWING CODE ///////////////////////////////////////
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// include MDNS
#ifdef ESP8266
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <ESPmDNS.h>
#endif

WiFiManagerParameter custom_field; // global param ( for non blocking w params )

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>


#include <HardwareSerial.h>
#include <TinyGPS.h>

//#include <Dusk2Dawn.h>

#include <NTPtimeESP.h>
#include <TimeLib.h>
#include <Ticker.h>


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


#include <EEPROM.h>
#include <ESP8266WebServer.h>

#include <WiFiClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>


//Needed for webupdate
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <CertStoreBearSSL.h>
BearSSL::CertStore certStore;
#include <time.h>


// WIfi Maneger
WiFiManager wm;

unsigned int  timeout   = 120; // seconds to run for
unsigned int  startTime = millis();
bool portalRunning      = false;
bool startAP            = true; // start AP and webserver if true, else start only webserver

// Included for NPT time
//#include <WiFiUdp.h>
//#include <HTTPClient.h>
//#include <HTTPUpdate.h>
//#include <WiFiClientSecure.h>

//const char* host = "ChickenDoor";
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE); 



IPAddress IP_SERVER(192, 168, 1, 1);

TinyGPS gps;
HardwareSerial SerialGPS(1);


#define ONE_WIRE_BUS 2
#define I2C_SCL 4
#define I2C_SCA 5

#define MPB_SET 0
#define MPB_MENU 2


#define MOTOR_IN1 12
#define MOTOR_IN2 13

#define ACC_EN 14

#define ACC_GPS 15
#define TX_GPS 1
#define RX_GPS 3
#define BUTTON_PIN_BITMASK 0x8004



// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
//ConfigStruct configs

///////////////////  PIN SETUP ////////////////////////////////////////////

//NTPtime NTPch("us.pool.ntp.org");
const char* serverTS = "api.thingspeak.com";
const char* hostTS = "api.thingspeak.com";
char APIkey[17] = "________________";
byte StateField = 1;
byte TempField = 2;
byte BatField = 3;
byte InField = 4;
/////////////////// SYSTEM SETTINGS //////////////////////////////////////
unsigned long RTC_UPDATE_ms = 1000;
unsigned long RTC_LAST = 0;
unsigned long Time_Record_ms = 1000; //1000000 after valid update;
unsigned long Time_Record_LAST = 0;
unsigned long DATA_Upload_ms = 900000;//1800000; // Every 15 mins
unsigned long DATA_Upload_LAST = 0;
unsigned long DOOR_CNTR_ms = 200;
unsigned long DOOR_CNTR_LAST = 0;
unsigned long ACC_timout_ms = 1800000; // Attempt to get GPS time for 30 mins
unsigned long ACC_LAST = 0;
unsigned long JAM_REV = 0;

unsigned long LCD_LOCK_ms = 8000;
unsigned long LCD_LOCK_LAST = 0;

unsigned long EEPROM_Upload_ms = 2000; //Min time between EEPROM Updates
unsigned long EEPROM_Upload_LAST = 0;

unsigned long   currentMillis = 0;


///////////////////// Variable Definition ///////////////////////////////

#define LCD_C     LOW
#define LCD_D     HIGH
// used in this example to print variables every 10 seconds
unsigned long printEntry;
String deviceName = "ChickenDoor";
String chipId;



const int numReadings = 1000;
float readings_V[numReadings];
int16_t readings_FB[numReadings];

byte aHour = 12;
String AM;

int holdI = 0;
bool IP = HIGH;
int MESunrise = 0;
int MESunset = 0;
byte Ayear = 0;
bool first = HIGH;
int second_cur, minute_cur, hour_cur, date_cur, month_cur, year_cur, temperature;
int iter = 0;
int CUR_time = 0;
int LAST_COM = 0;
bool blink_test = 0;
bool Century = false;
bool h12;
bool PM;
int OVER_LIM = 0;
bool going_up = 0;
bool going_down = 0;
int son = 500;
int JAMED = 0;
float BAT_LEV = 0;
float In_Level = 0;
int Correction_cnt = 0;
String ACT_S[2] = {"inactive", "active"};
String DIRC_S[3] = {"stoped", "going down", "going up"};
int REQtime;
String HTML, HTML2, Link;
bool DFirst = 1;
float TempF = 0;
float TempH;
int tcount;
int Time_Method=3;

int Feed_time = 0;
int Feed_length = 0;
int eepS;

unsigned long age;
int Year;
byte Month, Day, Hour, Minute, Second;
int ajH;

byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;

float flat, flon;
String tt;
int Move_Complete = 1;
int time_updated = 0;
int interTIME = 0;
int interTIMElast = 0;
int Time_ZONE = -5;
float LATT = 43.;
float LONG = -70.;
int After_Sunrise = 5 ;
int After_Sunset = 20;
int up_limit = 800;
int down_limit = 300;
Ticker secondTick;


volatile bool is_active = 0;
int Top = 0;
int Bot = 0;
byte is_top = 0;
byte is_bot = 0;
int DOWNREQUEST = 0;
int UPREQUEST = 0;
bool STOPREQUEST = 0;
int FB_READ;
int request_direc = 0; //1 - up requested 2 - Down Requested
byte DSE=0;
byte DSE_TV=0;
uint64_t sleepT;

String FirmwareVer = {
  "1.004"
};
#define URL_fw_Version "https://raw.githubusercontent.com/tinkydan/WiFiChickenDoor/main/bin_version.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/tinkydan/WiFiChickenDoor/main/ChickenDoorFW.bin"
const int httpsPort = 443;
const char* host = "raw.githubusercontent.com";

#define EEPROM_SIZE 65
#define uS_TO_S_FACTOR 1000000ULL


float f = 0;

NTPtime NTPch("ch.pool.ntp.org");
strDateTime dateTime;
WiFiClient client;

                 // Initialize IotAppStory
//AsyncWebServer serverV(80);

//

void setup() {


// WiFi Maneger Setup 
   // WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP    
    // put your setup code here, to run once:
   Serial.begin(74880);
    
    //reset settings - wipe credentials for testing
    //wm.resetSettings();
   // wm.addParameter(&custom_mqtt_server);
    wm.setConfigPortalBlocking(false);
   // wm.setSaveParamsCallback(saveParamsCallback);

    //automatically connect using saved credentials if they exist
    //If connection fails it starts an access point with the specified name
    if(wm.autoConnect("AutoConnectAP")){
        Serial.println("connected...yeey :)");
    }
    else {
        Serial.println("Configportal running");
    }


   
  u8g2.begin();
//Wire.begin(5,4);
  ADS.begin(5,4);
  ADS.setGain(0);
 // Wire.begin();
Update_Screen();
  
Serial.println("Initilizing");

char sz[32];

  dateTime = NTPch.getNTPtime(Time_ZONE, 0);
  tcount = 0;
  while (!dateTime.valid && tcount < 100) {
    tcount += 1;
    dateTime = NTPch.getNTPtime(Time_ZONE, 0);
  }
  if (dateTime.valid){
     if ((DSE == 1) || ((DSE == 2) && (BAT_LEV < 3.4)) || ((DSE == 3) && (In_Level < 1))){
      // Deep sleep enabled 
      DSE_TV=1;
     }
  }
  setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);





//  esp_task_wdt_init(60, true);
 // esp_task_wdt_add(NULL);
}

void loop() {
 
  // put your main code here, to run repeatedly:
  if (!is_active) {
        wm.process();
    // Check for Updates and things
  }
//  esp_task_wdt_reset(); // Resets if board locks up

  currentMillis = millis();

// Get status of the door
int sensorValue = analogRead(A0);
if (sensorValue<157){
  is_top = 0;
  is_bot = 0;
}
else if (sensorValue<328){
  is_top = 0;
  is_bot = 2;// UNPLUGGED
}
else if (sensorValue<403){
  is_top = 2;// UNPLUGGED
  is_bot = 0;
}
else if (sensorValue<505){
  is_top = 1;
  is_bot = 0;
}
else if (sensorValue<615){
  is_top = 0;
  is_bot = 1;
}
else if (sensorValue<731){
  is_top = 2;
  is_bot = 2;// UNPLUGGED
}
else if (sensorValue<846){
  is_top = 1;
  is_bot = 2;// UNPLUGGED
}
else if (sensorValue<956){
  is_top = 2;
  is_bot = 1;// UNPLUGGED
}
else {
  is_top = 1;
  is_bot = 1;// UNPLUGGED
}


  CUR_time = minute() + 60 * hour();
  if ((LAST_COM > 0) && (CUR_time < 5)) { //Resets last time commanded at midnight
    LAST_COM = 0;
  }


  if (blink_test) {
    if (!is_active) {

      if (is_bot) {
        delay(4000);
        UPREQUEST = 1;
      }
      else if (is_top) {
        delay(4000);
        DOWNREQUEST = 1;
      }
      else if (!is_top && !is_bot) {
        delay(4000);
        UPREQUEST = 1;
      }
    }
  }

  
  if (((((CUR_time)) - After_Sunset) > MESunset) && ((((CUR_time)) - After_Sunset) < (MESunset + 10)) ) {
    if (!is_bot && !OVER_LIM && ((LAST_COM + 1) < CUR_time)) {
      LAST_COM = CUR_time;
      DOWNREQUEST = 1;
      dateTime = NTPch.getNTPtime(Time_ZONE, 0);
      tcount = 0;
      while (!dateTime.valid && tcount < 2000) {
        tcount += 1;
        dateTime = NTPch.getNTPtime(Time_ZONE, 0);
      }
      if (dateTime.valid) {
        setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);
      }
    }
  }
  else if ( (((CUR_time) - After_Sunrise) > MESunrise) && (((CUR_time) - After_Sunrise) < (MESunrise + 10))) {
    if (!is_top && !OVER_LIM && ((LAST_COM + 1) < CUR_time)) {
      LAST_COM = CUR_time; // Makes sure this is only called once every min
      UPREQUEST = 1;
      dateTime = NTPch.getNTPtime(Time_ZONE, 0);
      tcount = 0;
      while (!dateTime.valid && tcount < 2000) {
        tcount += 1;
        dateTime = NTPch.getNTPtime(Time_ZONE, 0);
      }
      if (dateTime.valid) {
        setTime(dateTime.hour, dateTime.minute, dateTime.second, dateTime.day, dateTime.month, dateTime.year);
      }

    }
  }


  if (is_top && going_up && (is_top != is_bot)) { // Not sure it shoud not just stop if sensor is unpluged
    if (request_direc == 1) {
      STOPREQUEST = 1;
    }
    if (request_direc == 2) {
      DOWNREQUEST = 2;
    }
  }

  if (is_bot && going_down  && (is_top != is_bot)) { // Not sure it shoud not just stop if sensor is unpluged
    if (request_direc == 2) {
      STOPREQUEST = 1;
    }
    if (request_direc == 1) {
      UPREQUEST = 2;
    }
  }

//Wire.begin(5,4);
  FB_READ = -ADS.readADC_Differential_0_1();//analogRead(MOTOR_FB);
  // Wire.begin();

  if ((((FB_READ > up_limit) && (going_up)) || ((FB_READ > down_limit) && (going_down))) && (millis() + 9000 >= JAM_REV )) {
    delay(200);
   // Wire.begin(5,4);
    FB_READ = -ADS.readADC_Differential_0_1();

  //  Wire.begin();
    if (FB_READ > down_limit) {
      OVER_LIM++;
      if (going_down && (OVER_LIM > 3)) {
        STOPREQUEST = 1;
        JAMED = 1;

      }
      if (going_up && (OVER_LIM > 4)) {
        STOPREQUEST = 1;
        JAMED = 2;

      }
      else if (going_down) {
        UPREQUEST = 2;
        Serial.println("Jamed sending up");
      }
      else if (going_up) {
        DOWNREQUEST = 2;
        Serial.println("Jamed sending down");
      }
      JAM_REV = millis() + 10000;
    }
  }

  // reveses the motor after a jam if other position has not been reached

  if ((OVER_LIM > 0) && (JAM_REV > 0) && (millis() >= JAM_REV )) {
    JAM_REV = 0;
    if (going_down && (request_direc == 1)) {
      UPREQUEST = 2;
    }
    else if (going_up && (request_direc == 2)) {
      DOWNREQUEST = 2;
    }
  }


  //////////////////////// Excecute Close or open Commands ///////////////////////////
  if (DOWNREQUEST >= 1)
  {
    ACC_LAST = currentMillis;
    Serial.println("Going Down");
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(ACC_EN, HIGH);
//    digitalWrite(BL, HIGH);
    is_active = HIGH;
    going_down = HIGH;
    going_up = LOW;
    if (DOWNREQUEST == 1) {
      OVER_LIM = 0;
      JAMED = 0;
      DOWNREQUEST = 0;
      request_direc = 2;
    }
    if (DOWNREQUEST == 2) {
      delay(1000);
      DOWNREQUEST = -1;
    }


  }
  if (UPREQUEST >= 1)
  { ACC_LAST = currentMillis;
    Serial.println("Going Up");
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(ACC_EN, HIGH);

    is_active = HIGH;
    going_up = HIGH;
    going_down = LOW;
    if (UPREQUEST == 1) {
      OVER_LIM = 0;
      JAMED = 0;
      UPREQUEST = 0;
      request_direc = 1;
    }
    if (UPREQUEST == 2) {
      delay(100);
      UPREQUEST = -1;
    }

  }


  if (STOPREQUEST == 1)
  { STOPREQUEST = 0;
    request_direc = 0;
    Serial.println("Stopping");
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    Move_Complete = 1;
    going_up = LOW;
    going_down = LOW;
    OVER_LIM = 0;
    is_active = 0;

  }




  ////////////////////////// RTC UPDATE & SUN TIMES ///////////////////////////////////  /
  if (currentMillis - RTC_LAST >= RTC_UPDATE_ms )
  { RTC_LAST = currentMillis;
Update_Screen();
//    print_time();

    //Civil twilight sun down and rise times
//    Dusk2Dawn ME(LATT, LONG, Time_ZONE);
    second_cur = second();
    minute_cur = minute();
    hour_cur = hour();
    date_cur = day();
    month_cur = month();
    year_cur = year();


  //  MESunrise  = ME.sunrise(year_cur, month_cur, date_cur, false);//Calculating Civil Twilight times
 //   MESunset   = ME.sunset(year_cur, month_cur, date_cur, false);




  }
  ////////////////////////////////////////////////////////////////////////////


  ////////////////////////// DOOR CONTROL ////////////////////////////////////
  if (currentMillis - DOOR_CNTR_LAST >= DOOR_CNTR_ms)
  { DOOR_CNTR_LAST = currentMillis;

    TempH = sensors.getTempCByIndex(0) * 9 / 5 + 32;
    if (TempH > -150 & TempH != 0) {
      TempF = TempH;

    }
    sensors.requestTemperatures();
//Wire.begin(5,4); 
    ADS.setGain(16);
    int16_t val_01 = ADS.readADC_Differential_0_1();
    float volts_01 = -ADS.toVoltage(val_01);
    ADS.setGain(1);
    int16_t val_2 = ADS.readADC(2);
    int16_t val_3 = ADS.readADC(3);

    float f = ADS.toVoltage(1);  // voltage factor
    ADS.setGain(16);
  //  Wire.begin(); 
   // Serial.print("\tval_01: "); Serial.print(val_01 * 0.256 / 0.05); Serial.print("\t"); Serial.println(volts_01, 3);
  //  Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t'); Serial.println(val_2 * f, 3);
  //  Serial.print("\tAnalog3: "); Serial.print(val_3 ); Serial.print('\t'); Serial.println(val_3 * f, 3);
    // Serial.println();
    BAT_LEV = (double(val_3) - 293) / 886 * 3.77;//*0.780794+0.847166;
    BAT_LEV = (double(val_3)/13552*4.1);//*0.780794+0.847166;
   // BAT_LEV = map(BAT_LEV, 2.45, 4.32, 2.76, 4.22);
    In_Level = val_2 * 5.03 / 835 * 4.92 / 7.16;
    In_Level = val_2 * 5.1 / 13552;
    if (is_active) {
      for (int i = numReadings - 1; i > 0; i--)
      {
        readings_FB[i] = readings_FB[i - 1];
      }
      readings_FB[0] = -val_01 ;
    }
  }
  //////////////////////////////////////////////////////////////////////////////


  ////////////////////////// Time Update LOOP ////////////////////////////////////
  if (currentMillis - Time_Record_LAST >= Time_Record_ms || first ) {
    Time_Record_LAST = currentMillis;
    while (SerialGPS.available()) {
      if (gps.encode(SerialGPS.read())) { // process gps messages
        // when TinyGPS reports new data...

        gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
        char sz[32];

        if (age < 500) {
          // set the Time to the latest GPS reading

          first = LOW;
          /// Get current position
          gps.f_get_position(&flat, &flon, &age);
          Serial.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
          Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
          if ((flon != 0) || (flat != 0)) {
            time_updated += 1;
            Serial.println("Time Update: " + String(time_updated));
            sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ", Month, Day, Year, Hour, Minute, Second);
            Serial.println(sz);
            Serial.println(Time_ZONE);
            ajH = Time_ZONE + Hour;
            if (ajH < 0) {
              ajH += 24;
              Day += -1;
            }
            if (ajH > 23) {
              ajH += -24;
              Day += 1;
            }
            setTime(ajH, Minute, Second, Day, Month, Year);

           // Serial.println("Time adjusted: " + web_time());
            if ((abs(sqrt(sq(flat) + sq(flon)) - sqrt(sq(LATT) + sq(LONG)))) > 0.01) {
              LATT = flat;
              LONG = flon;
              Time_ZONE = int((LONG - 7.5) / 15) + 1; //get_tz(LATT,LONG);
              Serial.println("Location has changed recalculate location");
              EEPROM.begin(EEPROM_SIZE);
              EEPROM.put( 0 + eepS, LATT);
              EEPROM.put( 4 + eepS, LONG);
              EEPROM.put( 8 + eepS, Time_ZONE);
              EEPROM.commit();
              EEPROM.end();
              ajH = Time_ZONE + Hour;
              if (ajH < 0) {
                ajH += 24;
                Day += -1;
              }
              if (ajH > 23) {
                ajH += -24;
                Day += 1;
              }
              setTime(ajH, Minute, Second, Day, Month, Year);

            }

          }
        }
      }


    }

  }




  //////////////////////////////////////////////////////////////////////////////

  // Accesory Shutoff
  if ((time_updated > 30 && Move_Complete) || ((currentMillis - ACC_LAST >= ACC_timout_ms ) && Move_Complete)) {
    time_updated = 0;
    Move_Complete = 0;
    UploadTS();
//    digitalWrite(BL, LOW);
    digitalWrite(ACC_EN, LOW);
    // DEEP SLEEP
//    deep_sleep();
  }
  ////////////////////////// DATA UPLOAD ////////////////////////////////////
  if (((currentMillis - DATA_Upload_LAST >= DATA_Upload_ms)||DFirst) && !is_active) 
  { DATA_Upload_LAST = currentMillis;
  
  FirmwareUpdate();
   if ( DFirst == 0){
    UploadTS();}
     DFirst = 0;
    tt = WiFi.localIP().toString();
    if (tt == "0.0.0.0" ) {
      LCD_LOCK_LAST = millis();
     // lcd.setCursor(0, 0);
     // lcd.println("RECONNECTING!");

      // Connect WiFi | If you added multiple WiFi access points in de config pages. Connect to the strongest AP from the list.
     // IAS.WiFiConnect();


    }


if ((DSE_TV==1)&&!is_active){
      UploadTS();
     digitalWrite(ACC_EN, LOW);
    // DEEP SLEEP
   //  deep_sleep();
   
     
}

    



    for (int i = numReadings - 1; i > 0; i--)
    {
      readings_V[i] = readings_V[i - 1];
    }
  //  Serial.println("Battery level" + String(BAT_LEV));
    readings_V[0] = BAT_LEV;

//  deep_sleep();

  }
  //////////////////////////////////////////////////////////////////////////////


}
