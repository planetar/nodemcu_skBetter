/*
  dies ist ein Ast der skBett - PIO-migration, und zwar der eine, wo pubsubClient den mqConnect ohne exception absolviert.
  dieser Stand schmeisst die Exception, sobald ein attachinterrupt für PIR aktiv ist.
  FastLED und HTU21d fehlen hier noch.
  Was aber hier wunderbar funktioniert ist der OTA upload. 



*/

#include <Arduino.h>

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>
#include <config.h>


#include <Wire.h>

//#include <HTU21D.h>
//#define FASTLED_ESP8266_D1_PIN_ORDER
//#include <FastLED.h>


const char* version = "0.01.37";

#define SENSORNAME "skBett"

// sensor state & set
#define STATE_TOPIC    "sensor/skBett/state"
#define SET_TOPIC      "sensor/skBett/set"
#define DBG_TOPIC      "sensor/skBett/dbg"
#define STATE_TOPIC_BK "led/bk/state"
#define SET_TOPIC_BK    "led/bk/set"
#define STATE_TOPIC_FK  "led/fk/state"
#define SET_TOPIC_FK    "led/fk/set"


// clientID
int clientID;



/*********************************env-info************************************************/


const char* on_cmd = "ON";
const char* off_cmd = "OFF";



// timed_loop 
#define INTERVAL_0     60
#define INTERVAL_1    100
#define INTERVAL_2  10000
#define INTERVAL_3  60000
#define INTERVAL_4 180000

unsigned long time_0 = 0;
unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;


// debug messages
const int numMsg=80;
int msgCount=0;
String msg=".";
String arrMessages[numMsg];

///////////**********************/


#define I2C_PIN_SDA    D1
#define I2C_PIN_SCL    D2
#define LED_PIN_1      D5
#define LED_PIN_2      D6
#define PIR_PIN        D7

#define CHIPSET     WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS_1    150
#define NUM_LEDS_2    150

byte brightness = 18;

  
bool stateOn_1 = true;
bool startFade_1 = false; 
bool stateOn_2 = true;
bool startFade_2 = false;
 
int hueRange_1=8;
//int satRange_1=12;
int quart_1=113;
int direction_1=1;



 
// CHSV baseColor_1(11,210,230);
// CHSV currColor_1(15,210,230);
// CHSV baseColor_2(11,210,230);
// CHSV currColor_2(15,210,230);
int hueRange_2=8;
//int satRange_2=12;
int quart_2=87;
int direction_2=1;

unsigned int stepTime_1=50;
unsigned int stepTime_2=40;

const char* effect_1 = "rangeWave";
const char* effect_2 = "rangeWave";

String effectString_1 = "rangeWave";
String effectString_2 = "rangeWave";

// struct CRGB leds_1[NUM_LEDS_1];
// struct CRGB fadeTargets_1[NUM_LEDS_1];
// struct CRGB fadeNormals_1[NUM_LEDS_1];

// struct CRGB leds_2[NUM_LEDS_2];
// struct CRGB fadeTargets_2[NUM_LEDS_2];
// struct CRGB fadeNormals_2[NUM_LEDS_2];



unsigned long  INTERVAL_anim = 2000;
int animFactor = 1;
unsigned long time_anim = 0;


bool firstRun_1  = true;
bool firstRun_2  = true;


// struct CHSV roleSpool_1[60];

// struct CHSV roleSpool_2[60];



const int faderLength=30;
int arrFader_1[faderLength];
int newFade_1=0;

int arrFader_2[faderLength];
int newFade_2=0;

/******************************** GLOBALS for animations *******************************/

int loopCount_1=0;
int loopCount_2=0;
int animCount_1=0;
int animCount_2=0;
unsigned long lastLoop_1=millis();
unsigned long lastLoop_2=millis();

bool lockForAnim = false;

float hueStep_1,hueStep_2;
float hue_1,hue_2;

/*********************************/
float temp=0;
float hum=0;

bool flagPir  = false;
bool useHtu   = false;
bool scan_i2c = false;  // true;
String i2c_addr="";


//instances
WiFiClient espClient;
PubSubClient mqClient(espClient);


unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msgBuffer[MSG_BUFFER_SIZE];
long int value = 0;

// declarationen
void tuWas(); //dummy

void mqCallback(char* , byte* , unsigned int );
void readNews(char* , byte* , unsigned int );
void readNewsBK(char* , byte* , unsigned int );
void readNewsFK(char* , byte* , unsigned int );

void timed_loop();
void doReset(String );


void scani2c();
//mkIRAM_ATTR void handlePir();
void checkFlags();


void sendDbg(String );
void checkDebug();
void debug(String , boolean );
void sendState() ;

void setupWifi();
void setupMq();
void setupOta();
void mqConnect();



/************************************/

void setup() {

  debug(String(SENSORNAME)+" "+version,true);

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setupWifi();
  setupMq();

  setupOta();

   // Wire.begin(SDA, SCL);
  Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
  if (scan_i2c){
     scani2c();
  }
   // pinMode
  pinMode(PIR_PIN, INPUT);   // declare sensor as input
  
  //attachInterrupt(digitalPinToInterrupt(PIR_PIN), handlePir, RISING);  
}


void loop() {

  mqClient.loop();
  ArduinoOTA.handle();
  

  timed_loop();
  //checkFlags();
} 

void timed_loop() {
   if(millis() > time_0 + INTERVAL_0){
    time_0 = millis();

    //colorLoop();  
    
      //checkDebug();
       
  } 
  
  if(millis() > time_1 + INTERVAL_1){
    time_1 = millis();
    //tuWas();

    if (!mqClient.connected()) {
      mqConnect();
    }  
  }
   
  if(millis() > time_2 + INTERVAL_2){
    time_2 = millis();
    sendState();
  }
 
  if(millis() > time_3 + INTERVAL_3){
    time_3 = millis();
      

  }

  if(millis() > time_4 + INTERVAL_4){
    time_4 = millis();
        
        //  if (!lockForAnim){
        //   sendState();
          // sendState_1();
          // sendState_2();
      //}
  }
  
 

}


/*****************************************************/



void doReset(String msg){
  debug("about to restart "+msg,true);
  checkDebug();
  delay(50);
  ESP.restart();
}


void scani2c(){
  // 
  byte error, address; //variable for error and I2C address
  int nDevices;

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16){
        Serial.print("0");
        i2c_addr.concat("0");
      }
      Serial.print(address, HEX);
      i2c_addr.concat(String(address,HEX));
      i2c_addr.concat(" ");
      Serial.println("  ! ");
      Serial.println(i2c_addr);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //i2c_addr.concat(" | ");
}



// IRAM_ATTR void handlePir(){
//   flagPir=true;
// }

void checkFlags(){
 
  if (flagPir){
    sendState();    
    flagPir=false;

  }
}


/*****************************************************/



/*****************************************/


void mqCallback(char* topic, byte* payload, unsigned int length) {
  // Serial.print("Message arrived ");
  // Serial.println(topic);

    if (strcmp(topic, SET_TOPIC_BK) == 0) {
    readNewsBK(topic, payload, length);
  } else if (strcmp(topic, SET_TOPIC_FK) == 0) {
    readNewsFK(topic, payload, length);
  } else if (strcmp(topic, SET_TOPIC) == 0) {
    readNews(topic, payload, length);
  }
}

void readNews(char* topic, byte* payload, unsigned int length) {
  Serial.print("readNews  ");
  Serial.println(topic);
 
  JsonDocument doc;
  deserializeJson(doc, payload, length);

  // reset

  if (doc.containsKey("seppuku")) {
      String memento = doc["seppuku"];
      doReset(memento);
  }
  
  if (doc.containsKey("reset")) {
      
      doReset("reset!");
  }
  
}

void readNewsFK(char* topic, byte* payload, unsigned int length) {
  Serial.print("readNews  ");
  Serial.println(topic);
 
  JsonDocument doc;
  deserializeJson(doc, payload, length);

  // reset

  if (doc.containsKey("seppuku")) {
      String memento = doc["seppuku"];
      doReset(memento);
  }
  
  if (doc.containsKey("reset")) {
      
      doReset("reset!");
  }
  
}

void readNewsBK(char* topic, byte* payload, unsigned int length) {

  Serial.print("readNews  ");
  Serial.println(topic);
 
  JsonDocument doc;
  deserializeJson(doc, payload, length);

  // reset

  if (doc.containsKey("seppuku")) {
      String memento = doc["seppuku"];
      doReset(memento);
  }
  
  if (doc.containsKey("reset")) {
      
      doReset("reset!");
  }
  
}

void tuWas(){

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msgBuffer, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqClient.publish(STATE_TOPIC, msgBuffer);
  }
}


void sendState() {

  JsonDocument doc;
  doc["pir"] = (flagPir) ? on_cmd : off_cmd;
  if (useHtu){
    doc["temp"]=temp;
    doc["hum"]=hum;

  }
  doc["id"]   = clientID;
  doc["vers"] = version;

  char buffer[256];
  serializeJson(doc, buffer);

  mqClient.publish(STATE_TOPIC, buffer, true);
  doc.clear();
}


///////////////////////////////////////////////////////////////////////////////

// send a message to mq
void sendDbg(String msg){
  JsonDocument doc;
 
  doc["dbg"]=msg;
  

  char buffer[512];
  size_t n = serializeJson(doc, buffer);

  mqClient.publish(DBG_TOPIC, buffer, n);
}

// called out of timed_loop async
void checkDebug(){
  if (msgCount>0){
    
    String message = arrMessages[0];

     for (int i = 0; i < numMsg-1; i++) {
      arrMessages[i]=arrMessages[i+1];
    }
    arrMessages[numMsg-1]="";
    msgCount--;
    sendDbg(message);
  }
  
  
}

// stuff the line into an array. Another function will send it to mq later
void debug(String dbgMsg, boolean withSerial){
  //Serial << "dbgMsg: " << dbgMsg <<  "\n";
  
  if (withSerial) {
    Serial.println( dbgMsg );
  }
  if (msgCount<numMsg){
    arrMessages[msgCount]=dbgMsg;
    msgCount++;
  }
  
}


/*****************************************/

void setupWifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFISSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFISSID, WIFIPASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress ip = WiFi.localIP();
  clientID = ip[3];

  msg = "WiFi connected, local IP: "+ip.toString();
  
}


void mqConnect() {
  // Loop until we're reconnected
  if (!mqClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // if ( mqttClient.connect("minimal","spy","autan")) {
    if (mqClient.connect(SENSORNAME,MQTT_USERNAME,MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqClient.publish(DBG_TOPIC, "hello world");
      // ... and resubscribe
      mqClient.subscribe(SET_TOPIC);
      mqClient.subscribe(SET_TOPIC_BK);
      mqClient.subscribe(SET_TOPIC_FK);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}

void setupMq(){

  mqClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqClient.setCallback(mqCallback);
}


void setupOta(){

  //OTA SETUP
  ArduinoOTA.setPort(OTAPORT);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(SENSORNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTAPASSWORD);

  ArduinoOTA.onStart([]() {
    debug("Starting OTA",false);
  });
  ArduinoOTA.onEnd([]() {
    debug("End OTA",false);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debug("OTA Auth Failed",false);
    else if (error == OTA_BEGIN_ERROR) debug("OTA Begin Failed",false);
    else if (error == OTA_CONNECT_ERROR) debug("OTA Connect Failed",false);
    else if (error == OTA_RECEIVE_ERROR) debug("OTA Receive Failed",false);
    else if (error == OTA_END_ERROR) debug("OTA End Failed",false);
  });
  ArduinoOTA.begin();
  
}
