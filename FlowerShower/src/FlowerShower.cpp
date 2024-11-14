/* 
 * Project FlowerShower
 * Author: BH
 * Date: 11-11-2024
 */

#include "Particle.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_BME280.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "IoTClassroom_CNM.h"// includes Button.h and Colors.h and wemo.h
#include "credentials.h"
//#include "Air_Quality_Sensor.h"
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterButton");  
Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomData");
Adafruit_MQTT_Publish pubFeedSoil = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilMoisture");

Adafruit_BME280 bme;
IoTTimer PumpTimer;//how long the pump works
IoTTimer WaitCheckTimer;//wait and then check the soil moisture

//OLED
const int OLED_RESET=-1;
Adafruit_SSD1306 display(-1);

//VARIABLES
float humidRH;
float pressPA;
float tempC;
float tempF;//based from tempC
float inHg;//based from pressPA

float currTemp;
float currPresr;
float currHum;
//TIMERS
int currentTime;
int lastSecond;
int vSeconds;

String dateTime, timeOnly;//for timestamp
unsigned int lastTime;

//BME
int hexAddress = 0X76;
float status;
//MOISTURE
int moistureRead;
bool giveDrink;
//ADA
int subValue;
//int subValue2;

//PUMP
 const int PUMPPIN = D15;
 //AIR
 //AirQualitySensor sensor(A0);

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
void pumpWater();
void ShowEnviro();
void PrintOLED();

SYSTEM_MODE(AUTOMATIC);//online

//SETUP***************************
// setup() runs once, when the device is first turned on
void setup() {
  pinMode(A1,INPUT);
  moistureRead = analogRead(A1);
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  // // Connect to Internet but not Particle Cloud
  // WiFi.on();
  // WiFi.connect();
  // while(WiFi.connecting()) {
  //   Serial.printf(".");
  // }
  // Serial.printf("\n\n");

  // Setup MQTT subscription
  mqtt.subscribe(&subFeed);
  // Initialize the BME280
  status = bme.begin(hexAddress);
  if (status == false){
      Serial.printf("BME280 at address 0x%02X failed to start", hexAddress);
  }
  pinMode(PUMPPIN,OUTPUT);
  
    //TIME
    Time.zone(-7); // MST =-7, MDT =-6
    Particle.syncTime();
  //OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
    display.clearDisplay();
    display.display();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.printf("READY");
    display.display();
    //Air Quality Sensor:
    //if (sensor.init()) {Serial.println("Sensor ready.");} else {Serial.println("Sensor ERROR!");}
  WaitCheckTimer.startTimer(600000);//ten minutes
}

void loop() {
  //ten minute timer
  if (WaitCheckTimer.isTimerReady()){
    moistureRead = analogRead(A1);
    if (moistureRead>=2900){
        Serial.printf("M: %i\n",moistureRead);
        PumpTimer.startTimer(500);
        digitalWrite(PUMPPIN,HIGH);
        WaitCheckTimer.startTimer(600000); 
    }
    }
    if(PumpTimer.isTimerReady()){
      digitalWrite(PUMPPIN,LOW);
      //delay(10);
      PrintOLED();
    }

  MQTT_connect();
  MQTT_ping();
  //Receiving data from BME280
    tempC = bme.readTemperature(); //degrees C
    pressPA = bme.readPressure();
    humidRH = bme.readHumidity();
    //ShowEnviro();
    //Serial.printf("T: %i\nP: %i\nH: %i\n",tempF,pressPA,humidRH);
    currentTime = millis();
    if ((currentTime - lastSecond)>1000*vSeconds){lastSecond = millis();
        // Serial.printf("Temp is %f\nPres is %f\nHumidity is %f\n", tempC,pressPA,humidRH);
      tempF = tempC * 1.8 + 32;
      inHg = pressPA / 3386.39;

    //if (currTemp != tempF){Serial.printf("Temperature is %f\n ", tempC); delay(500);}
    //if (inHg != currPresr){Serial.printf("Pressure is %f\n ", pressPA); delay(500);}
    //if (currHum != humidRH){Serial.printf("Humidity is %f\n ", humidRH); delay(500);}

  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &subFeed) {
      subValue = atoi((char *)subFeed.lastread);
      Serial.printf("SUBVALUE = %i \n", subValue);
      //BUTTON ACTION
      if (subValue == 1) {
        Serial.printf("HERE - Button is pressed \n");
        Serial.printf("T: %i\nP: %i\nH: %i\n",tempF,pressPA,humidRH);
        PumpTimer.startTimer(500);
        digitalWrite(PUMPPIN,HIGH);
        //PrintOLED();

        moistureRead = analogRead(A1);
        //dateTime = Time.timeStr(); //Current Date and Time from Particle Time class
        //timeOnly = dateTime.substring(11,19);
        //PRINT THE DISPLAY SCREEN .... moved prep code to a function printOLED
        // PrintOLED();
      }
    }
  }

//KEEP CURRENT SETTINGS TO COMPARE WITH NEW READINGS
    currTemp = tempF;
    currPresr = inHg;
    currHum = humidRH;
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");

  //PUBLISH TO ADAFRUIT ROOM DATA
    if((millis()-lastTime > 60000)) {
    if(mqtt.Update()) {
      pubFeed.publish(humidRH);
      pubFeedSoil.publish(moistureRead);
      Serial.printf("Sending to Ada: Soil = %i \n", moistureRead);
      } 
    lastTime = millis();
  }
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
void PrintOLED(){

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.printf("M: %i\n\n",moistureRead);
  display.setTextSize(1);
  //display.setCursor(0,26);
  dateTime = Time.timeStr(); //Current Date and Time from Particle Time class
  timeOnly = dateTime.substring(11,19); //Extract the Time from the DateTime String
  display.printf("T: %s\n",timeOnly.c_str());
  display.display();
  Serial.printf("Now \n");

}
void PumpWater(){
    PumpTimer.startTimer(500);
    digitalWrite(PUMPPIN,HIGH);
}
