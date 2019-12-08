///*
// Name:		MiniDashboard.ino
// Created:	2019-11-30
// Author:	ohminy.com
//*/

#define FS_NO_GLOBALS
#include "FS.h"
#ifdef ESP32
  #include "SPIFFS.h" // ESP32 only
#endif
#include <ESP8266WiFi.h>
#include "FS.h"
#include <WiFiClient.h>
#include <time.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "FSWebServerLib.h"
#include <Hash.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#include <TJpg_Decoder.h>
//---- Button ----
#include <ButtonKing.h>
//#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t
#include <RTClib.h>

/************ MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define mqtt_server "192.168.219.100"
#define mqtt_port 1883
#define mqtt_user "homeassistant" 
#define mqtt_password "homeassistant"

//#define state_topic "/state"        //uptime,wifi,Backlight,Light,LDR
//#define set_topic "/set"            //Backlight,Light,시간,날씨,온도,배터리,restart
String state_topic = "/state"; //uptime,wifi,Backlight,Light,LDR
String set_topic = "/set";     //Backlight,Light,시간,날씨,온도,배터리,restart

WiFiClient espClient;
PubSubClient client(espClient);

int Dust = 0;
const char* Weather = "";
//const char* Weather_old = "init";
int Temperature = 22;
int Battery = 99;
bool Washer1 = false;
bool Washer2 = false;
bool Washer3 = false;

bool startOn = true;
bool stateOn = false;
const char* on_cmd = "ON";
const char* off_cmd = "OFF";

/******************************** Pin Definitions (nodeMCU) ***************************************/
const uint8_t LIGHT_SENSOR_PIN = A0;    //A0
const uint8_t BACKLIGHT_PIN = D0;       //D1 LED
                                        //D1 SDA i2c
                                        //D2 SCL i2c               /Default AP button
                                        //D3 DC RS AO
const uint8_t TOUCH_SENSOR_PIN = D4;    //D4 RST to RESET
                                        //D5 SCLK
const uint8_t AP_SWITCH_PIN = D6;       //D6 MISO
                                        //D7 MOSI
                                        //D8 CS
                                        //


/**************************** BACKLIGHT DEFINITIONS *******************************************/
int dimCycle[2] = {200,800}; //just 2 level ( on & off )
int dimValue = 1;
int dimValue_old = 0;

/**************************** TOUCH SENSOR DEFINITIONS *******************************************/
ButtonKing button1(TOUCH_SENSOR_PIN, true);

/**************************** LIGHT SENSOR DEFINITIONS *******************************************/
#define LDRmax 600  //어두으면 커짐, 이거보다 크면 night mode
#define LDRmin 500  //밝으면 작아짐, 이거보다 작으면 day mode
byte newLDRState =0;
byte oldLDRState =0;
int oldLDR =1000;
int newLDR =0;
float calcLDR;
float diffLDR = 25;

/************************************ time & clock *********************************************/
boolean initial = 1;
uint32_t targetTime = 0;       // for next 1 second timeout
byte omm = 99;
byte xcolon = 0;

// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
//uint8_t hh=conv2d(__TIME__);
//uint8_t mm=conv2d(__TIME__+3);
//uint8_t ss=conv2d(__TIME__+6);
uint8_t hh = 99;
uint8_t mm = 99;
uint8_t ss = 99;

RTC_DS1307 RTC;
//RTC_DS3231 RTC;
char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
byte xpos_init = 12;
byte ypos_init = 10;

uint8_t Interval_value = 99;

/***********************************  UPTIME (by second) ************************************/
long Day=0;
int Hour =0;
int Minute=0;
int Second=0;
int HighMillis=0;
int Rollover=0;

String uptime_string;

/*******************************  WiFi Check ***************************************/
int wifi_old = 0;


/******************************** Global valiables ****************************************/
const int BUFFER_SIZE = 300;
#define MQTT_MAX_PACKET_SIZE 512



unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 10000;           // interval at which to blink (milliseconds)





//####################################################################################################
// Setup
//####################################################################################################
void setup() {

	// WiFi is started inside library
	SPIFFS.begin(); // Not really needed, checked inside library and started if needed
	ESPHTTPServer.begin(&SPIFFS);
	/* add setup code here */

//Touch Sensor Initialize
  pinMode(TOUCH_SENSOR_PIN, INPUT_PULLUP);
  Serial.println("Touch Sensor Initialzed");
//  button1.setClick(buttonPressed);
//  button1.setShortClickStart(buttonHeld);

//Light Sensor Initialize
  pinMode(LIGHT_SENSOR_PIN, INPUT_PULLUP);
  Serial.println("Light Sensor Initialzed");

//TFT Initialize
  pinMode( BACKLIGHT_PIN, OUTPUT );
//  analogWrite( BACKLIGHT_PIN, dimCycle[dimValue] );
  tft.begin();
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
//  BackLightSet(dimValue);
  Serial.println("TFT Initialzed");

  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  TJpgDec.setJpgScale(1);
  // The byte order can be swapped (set true for TFT_eSPI)
  TJpgDec.setSwapBytes(true);

//MQTT Initialize
  client.setServer(mqtt_server, mqtt_port);

// callbacks
  button1.setClick(buttonPressed);
  TJpgDec.setCallback(tft_output);
  client.setCallback(callback);  
  reconnect();

  rtc_setup();
  uptime_check();
  clock_check();
  ldr_check();
  display_setup();

//Setup Done
  Serial.println("Ready");
  Serial.print("AP_Mode : "); Serial.println(!digitalRead(AP_ENABLE_BUTTON));
}

void loop() {
	// DO NOT REMOVE. Attend OTA update from Arduino IDE
	ESPHTTPServer.handle();

  /* add main program code here */

  button1.isClick(); //Touch check
  delay(100);
  client.loop();     // have to call at least 30 seconds
//  reconnect();       //MQTT check
  ldr_check();       //LDR check
  uptime_check();    //uptime check
  wifi_check();      //wifi check
  rtc_check();
  clock_check();

  initial = 0;
}


/*uint32_t syncProvider()
{
  return RTC.now().unixtime();  //either format works
}*/

/********************************* RTC Check *************************************/
void rtc_setup() {
  if(!RTC.begin()){
      Serial.println("Couldn't find RTC");
  }
  else {
    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
    }
    else {
//      rtc_check();
    }
  }
}

/********************************* RTC Check *************************************/
void rtc_check() { 
  DateTime now = RTC.now();
  if(WiFi.status() == WL_CONNECTED) {
    if(now.day() != Interval_value) { //wifi mode 일때 매 시간 1번 동기화
      Interval_value = now.day();
      Serial.print("Wifi connected. RTC adjusting.....");
      RTC.adjust(DateTime(year(),month(),day(),hour(),minute(),second())); //set the RTC from the system time in DateTime_t
      print_datetime();
      Serial.println("Done.");
    }
  }
}

/********************************* print_datetime *************************************/
void print_datetime() {
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
//  Serial.println("");
}

/********************************* clock_check *************************************/
void clock_check() {
  byte xpos = xpos_init;
  byte ypos = ypos_init;
  
  String a[6];
  DateTime now = RTC.now(); 

  if (now.second() != ss) {  //매초
    ss = now.second();
    if (now.minute() != mm) {       // 매분 - 전체 시간 표시
      mm = now.minute();
      a[0]=String(now.second(), DEC); 
      a[1]=String(now.minute(), DEC); 
      a[2]=String(now.hour(),   DEC);  
      a[3]=String(now.day(),    DEC); 
      a[4]=String(now.month(),  DEC); 
      a[5]=String(now.year()-2000,   DEC);  
      if(now.second()<10)      a[0]="0"+a[0];
      if(now.minute()<10)      a[1]="0"+a[1];
      if(now.hour()  <10)      a[2]="0"+a[2];
      if(now.day()   <10)      a[3]="0"+a[3];
      if(now.month() <10)      a[4]="0"+a[4];
      if(now.year()  <10)      a[5]="0"+a[5];
      
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor (4, 2);
      tft.print(a[5]+" "+a[4]+"/"+a[3]+" "+daysOfTheWeek[now.dayOfTheWeek()]); // This uses the standard ADAFruit small font

//      tft.setCursor (0, 0);
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);  // Leave a 7 segment ghost image, comment out next line!
      // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
      tft.drawString("88:88",xpos,ypos,4); // Overwrite the text to clear it
//      tft.setTextColor(TFT_WHITE, TFT_BLACK);

//      tft.fillRect(50,50,250,130, TFT_BLACK); // Clear information
      tft.setTextColor(TFT_WHITE,TFT_BLACK);
      xpos += tft.drawString(a[2], xpos, ypos, 4);// Print the string name of the font
      xcolon=xpos;
      xpos+= tft.drawChar(':',xpos,ypos,4);
      tft.drawString(a[1], xpos, ypos, 4);// Print the string name of the font
//      tft.drawString(a[0], xpos+10, ypos, 4);// Print the string name of the font

      print_datetime(); Serial.println("");
    }
  
    if (ss%2) {            //매초 : 점멸
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      xpos+= tft.drawChar(':',xcolon,ypos,4);
    }
    else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawChar(':',xcolon,ypos,4);
    }
  }
}

/********************************* uptime Check *************************************/
void uptime_check(){
  //** Making Note of an expected rollover *****//   
  if(millis()>=3000000000){ 
    HighMillis=1;
  }
  //** Making note of actual rollover **//
  if(millis()<=100000&&HighMillis==1){
    Rollover++;
    HighMillis=0;
  }
  long secsUp = millis()/1000;
  Second = secsUp%60;
  Minute = (secsUp/60)%60;
  Hour = (secsUp/(60*60))%24;
  Day = (Rollover*50)+(secsUp/(60*60*24));  //First portion takes care of a rollover [around 50 days]

  uptime_string = String(Day) + " days, " +
                  (Hour < 10 ? "0" : "") + String(Hour) + ":" +
                  (Minute < 10 ? "0" : "") + String(Minute) + ":" +
                  (Second < 10 ? "0" : "") + String(Second);
}

void display_setup() {
//Wifi 세기 출력
//    tft.drawRect (0, 10, 10, 25, TFT_DARKGREY);
    wifi_check();

//미세먼지 출력
    TJpgDec.drawFsJpg(0, 36, "/dust/0.jpg");

//날씨 출력
    tft.drawRect (82, 2, 40, 40, TFT_DARKGREY);
    TJpgDec.drawFsJpg(82, 2, "/weather/init.jpg");

//온도 출력      
//    tft.drawRect (82, 44, 40, 40, TFT_DARKGREY);
    temperature_check();

//배터리 상태 출력      
    tft.drawRect (82, 86, 40, 40, TFT_DARKGREY);
    battery_check();

//세탁기 상태 출력      
    TJpgDec.drawFsJpg(124,  2, "/washer/washer1_off.jpg");
    TJpgDec.drawFsJpg(124, 44, "/washer/washer2_off.jpg");
    TJpgDec.drawFsJpg(124, 86, "/washer/dryer1_off.jpg");

    Serial.println("Image Draw Done.");
}


/********************************* BUTTON PRESSED *************************************/
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // This might work instead if you adapt the sketch to use the Adafruit_GFX library
  // tft.drawRGBBitmap(x, y, bitmap, w, h);

  // Return 1 to decode next block
  return 1;
}

/********************************* BUTTON PRESSED *************************************/
void buttonPressed() {
  Serial.println("Touch Sensor Pressed");
  BackLightDim();
}

/*********************************** BUTTON Held *************************************/
void buttonHeld() {
  Serial.println("Touch Sensor Held");
//  digitalWrite(LED_LIGHT_PIN, !digitalRead(LED_LIGHT_PIN));
}

/*********************************** BACKLIGHT DIM *************************************/
void BackLightDim() {
  dimValue = (++dimValue>4) ? 0 : dimValue;
  BackLightSet(dimValue);
}

void BackLightSet(int dimValue) {
  if ( !(dimValue == dimValue_old) ) {
    analogWrite(BACKLIGHT_PIN, dimCycle[dimValue]);
    Serial.print("Backlight On");
    dimValue_old = dimValue;  
  }
  else {
    Serial.print("Backlight off4]");
  }
}

void BackLightSetp(int dimValuep) {
  analogWrite(BACKLIGHT_PIN, dimValuep);
  Serial.print("Set = "); Serial.print(dimValuep);
}

/********************************** MQTT *****************************************/

//const char* sCname = sC.c_str();

void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ESPHTTPServer._config.deviceName.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      char* pBuf = (char*)ESPHTTPServer._config.deviceName.c_str();
      String pBuf_s = pBuf + set_topic;
      Serial.print("sC.deviceName.c_str() : "); Serial.println( ESPHTTPServer._config.deviceName.c_str() );
      Serial.print("Set topic : "); Serial.println( pBuf_s );
      client.subscribe( pBuf_s.c_str() );
      sendState();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    delay(5000);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.print("Payload : "); Serial.println(message);

  if (!processJson(message)) {return;}

  sendState();
}



/********************************** START PROCESS JSON*****************************************/
/********************** Backlight,Dust, Weather,Temperature,Battery, Washer, Restart ****************************/
bool processJson(char* message) {
  StaticJsonDocument<BUFFER_SIZE> doc;
//  JsonObject root = doc.to<JsonObject>();
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(error.c_str());
    return false;
  }

  if (doc.containsKey("Dust")) {
    Serial.print("old Dust is "); Serial.println(Dust);
    if (Dust != (int)doc["Dust"]) {
      Dust = doc["Dust"];
      Serial.print("new Dust is "); Serial.println(Dust);
      if      (Dust == 0)TJpgDec.drawFsJpg(0, 36, "/dust/0.jpg");
      else if (Dust == 1)TJpgDec.drawFsJpg(0, 36, "/dust/1.jpg");
      else if (Dust == 2)TJpgDec.drawFsJpg(0, 36, "/dust/2.jpg");
      else if (Dust == 3)TJpgDec.drawFsJpg(0, 36, "/dust/3.jpg");
      else if (Dust == 4)TJpgDec.drawFsJpg(0, 36, "/dust/4.jpg");
      else if (Dust == 5)TJpgDec.drawFsJpg(0, 36, "/dust/5.jpg");
      else if (Dust == 6)TJpgDec.drawFsJpg(0, 36, "/dust/6.jpg");
      else if (Dust == 7)TJpgDec.drawFsJpg(0, 36, "/dust/7.jpg");
      else if (Dust == 8)TJpgDec.drawFsJpg(0, 36, "/dust/8.jpg");
    }
  }

  if (doc.containsKey("Weather")) {
    Weather = doc["Weather"];
    if      (strcmp(Weather,"clear-day")          ==0)TJpgDec.drawFsJpg(82, 2, "/weather/day.jpg");
    else if (strcmp(Weather,"clear-night")        ==0)TJpgDec.drawFsJpg(82, 2, "/weather/night.jpg");
    else if (strcmp(Weather,"rain")               ==0)TJpgDec.drawFsJpg(82, 2, "/weather/rainy-5.jpg");
    else if (strcmp(Weather,"snow")               ==0)TJpgDec.drawFsJpg(82, 2, "/weather/snowy-6.jpg");
    else if (strcmp(Weather,"sleet")              ==0)TJpgDec.drawFsJpg(82, 2, "/weather/rainy-6.jpg");
    else if (strcmp(Weather,"wind")               ==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy.jpg");
    else if (strcmp(Weather,"fog")                ==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy.jpg");
    else if (strcmp(Weather,"cloudy")             ==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy.jpg");
    else if (strcmp(Weather,"partly-cloudy-day")  ==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy-day-3.jpg");
    else if (strcmp(Weather,"partly-cloudy-night")==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy-night-3.jpg");
    else if (strcmp(Weather,"hail")               ==0)TJpgDec.drawFsJpg(82, 2, "/weather/rainy-7.jpg");
    else if (strcmp(Weather,"lightning")          ==0)TJpgDec.drawFsJpg(82, 2, "/weather/thunder.jpg");
    else if (strcmp(Weather,"thunderstorm")       ==0)TJpgDec.drawFsJpg(82, 2, "/weather/thunder.jpg");
    else if (strcmp(Weather,"windy-variant")      ==0)TJpgDec.drawFsJpg(82, 2, "/weather/cloudy.jpg");
  }

  if (doc.containsKey("Temperature")) {
    Temperature = (int)doc["Temperature"];
    temperature_check();
  }

  if (doc.containsKey("Battery")) {
    Battery = (int)doc["Battery"];
    battery_check();
  }

  if (doc.containsKey("Washer1")) {
    if (strcmp(doc["Washer1"], on_cmd) == 0) {
      Washer1 = true;
      TJpgDec.drawFsJpg(124,  2, "/washer/washer1_on.jpg");
    }
    else if (strcmp(doc["Washer1"], off_cmd) == 0) {
      Washer1 = false;
      TJpgDec.drawFsJpg(124,  2, "/washer/washer1_off.jpg");
    }
  }
  if (doc.containsKey("Washer2")) {
    if (strcmp(doc["Washer2"], on_cmd) == 0) {
      Washer2 = true;
      TJpgDec.drawFsJpg(124, 44, "/washer/washer2_on.jpg");
    }
    else if (strcmp(doc["Washer2"], off_cmd) == 0) {
      Washer2 = false;
      TJpgDec.drawFsJpg(124, 44, "/washer/washer2_off.jpg");
    }
  }
  if (doc.containsKey("Washer3")) {
    if (strcmp(doc["Washer3"], on_cmd) == 0) {
      Washer3 = true;
      TJpgDec.drawFsJpg(124,  86, "/washer/dryer1_on.jpg");
    }
    else if (strcmp(doc["Washer3"], off_cmd) == 0) {
      Washer3 = false;
      TJpgDec.drawFsJpg(124,  86, "/washer/dryer1_off.jpg");
    }
  }

  if (doc.containsKey("Restart")) {
    ESP.restart();
  }

  return true;
}

void temperature_check() {
  String a;

  if ( Temperature < 0 ) {
    a = String(-Temperature, DEC);
    if(-Temperature <10) a = "0" + a;
    tft.setTextColor(0x39DF, TFT_BLACK); //BLUE 0x001F
    tft.drawString("-",85,44,2);
    tft.drawString(a,87,55,4);
    tft.drawRect (82, 44, 40, 40, TFT_DARKGREY);
  }
  else {
    a = String(Temperature, DEC);
    if(Temperature <10) a = "0" + a;
    tft.setTextColor(0xF9C7, TFT_BLACK); //RED 0xF800
    tft.drawString("+",85,44,2);
    tft.drawString(a,87,55,4);
    tft.drawRect (82, 44, 40, 40, TFT_DARKGREY);
  }
}

void battery_check() {
  if (Battery <= 30 ) tft.setTextColor(0xF9C7, TFT_BLACK);
  else if (Battery <= 60 ) tft.setTextColor(0xFDA0, TFT_BLACK);
  else tft.setTextColor(0x07E0, TFT_BLACK); // Green

  if (Battery >= 100 ) Battery = 99;
  tft.drawString((String)Battery,87,95,4);
}

void wifi_check() {
  if(WiFi.status() == WL_CONNECTED) {
    if(checkBoundSensor(WiFi.RSSI(), wifi_old, 10)) {
      wifi_old = WiFi.RSSI();
      if ( WiFi.RSSI() < -60 )      TJpgDec.drawFsJpg(0, 10, "/WiFi/3.jpg");
      else if ( WiFi.RSSI() < -40 ) TJpgDec.drawFsJpg(0, 10, "/WiFi/2.jpg");
      else                          TJpgDec.drawFsJpg(0, 10, "/WiFi/1.jpg");
    }
  }
  else {
    if(checkBoundSensor(WiFi.RSSI(), wifi_old, 10)) {
      wifi_old = WiFi.RSSI();
      TJpgDec.drawFsJpg(0, 10, "/WiFi/0.jpg");       
    }
  }
}


/********************************** START SEND STATE*****************************************/
/************************** uptime,wifi,Backlight,LDR *********************************/
void sendState() {
  StaticJsonDocument<BUFFER_SIZE> doc;
  JsonObject root = doc.to<JsonObject>();

  String a;
  if (startOn == true) {
    a = "ON";
    root["Start"] = a;
  }
  else a = "OFF";
  root["Uptime"] = uptime_string;
  root["WiFi"] = WiFi.RSSI();
  root["LDR"] = newLDR;

  startOn = false;

  char buffer[BUFFER_SIZE];
//  root.printTo(buffer, sizeof(buffer));
  serializeJson(doc, buffer);
  char* pBuf = (char*)ESPHTTPServer._config.deviceName.c_str();
  String pBuf_s = pBuf + state_topic;
  Serial.print("State topic : "); Serial.println( pBuf_s );
  Serial.println(buffer);
  client.publish( pBuf_s.c_str(), buffer, true);
}

/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

/*********************************** SOFTWARE RESET **************************************/
void software_Reset() { // Restarts program from beginning but does not reset the peripherals and registers
  Serial.print("resetting");
  delay(1000);
  ESP.reset(); 
}

void ldr_check(){

  newLDR = analogRead(LIGHT_SENSOR_PIN); //LDR check

  if ( (newLDR > LDRmax) && (oldLDR < LDRmin) ) {
    newLDRState = 0;
    Serial.print("LDR : "); Serial.print(oldLDR); Serial.print(" -> "); Serial.println(newLDR);
    oldLDR = newLDR;
    oldLDRState = newLDRState;
    dimValue = 0;
    BackLightSet(dimValue);
    sendState();
  }
  else if ( (newLDR < LDRmin) && (oldLDR > LDRmax) ) {
    newLDRState = 0;
    Serial.print("LDR : "); Serial.print(oldLDR); Serial.print(" -> "); Serial.println(newLDR);
    oldLDR = newLDR;
    oldLDRState = newLDRState;
    dimValue = 1;
    BackLightSet(dimValue);
    sendState();
  }
}
