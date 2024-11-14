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
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterButton");  
//Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantData");
IoTTimer PumpTimer;

Adafruit_BME280 bme;

//VARIABLES
float humidRH;
float pressPA;
float tempC;
float tempF;
float inHg;

float currTemp;
float currPresr;
float currHum;
//TIMER
int currentTime;
int lastSecond;
int vSeconds;
//static unsigned int plast;

//BME
int hexAddress = 0X76;
float status;
//MOISTURE
int moistureRead;
bool giveDrink;
//ADA
//float subValue,pubValue;//original code
int subValue;
//int subValue2;

//PUMP
 const int PUMPPIN = D15;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
void pumpWater();
void ShowEnviro();

SYSTEM_MODE(SEMI_AUTOMATIC);



//SETUP***************************
// setup() runs once, when the device is first turned on
void setup() {
  pinMode(A1,INPUT);
  moistureRead = analogRead(A1);
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  // Setup MQTT subscription
  mqtt.subscribe(&subFeed);
 // mqtt.subscribe(&subFeed2);
  // Initialize the BME280
  status = bme.begin(hexAddress);
  if (status == false){
      Serial.printf("BME280 at address 0x%02X failed to start", hexAddress);
  }
  pinMode(PUMPPIN,OUTPUT);
  //pumpWater(); 
}

void loop() {
  moistureRead = analogRead(A1);
  Serial.printf("M: %i\n",moistureRead);
  if (moistureRead>=3100){
    //Serial.printf("Thirsty plant alert \n");
    PumpTimer.startTimer(500);
    digitalWrite(PUMPPIN,HIGH);
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
        // Serial.printf("Temperature is %f\n ", tempC);
        // Serial.printf("Pressure is %f\n ", pressPA);
        // Serial.printf("Humidity is %f\n ", humidRH);
      tempF = tempC * 1.8 + 32;
      inHg = pressPA / 3386.39;

    if (currTemp != tempF){
      // Serial.printf("Temperature is %f\n ", tempC);
      // delay(500);
    }
    if (inHg != currPresr){
      // Serial.printf("Pressure is %f\n ", pressPA);
      // delay(500);
    }
    if (currHum != humidRH){
      // Serial.printf("Humidity is %f\n ", humidRH);
      // delay(500);
    }

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
        //pumpWater(); 
        PumpTimer.startTimer(500);
        digitalWrite(PUMPPIN,HIGH);
      }
      else {
        digitalWrite(PUMPPIN,LOW);
      }

    }
    // if (subscription == &subFeed2) {
    //   subValue2 = atoi((char *)subFeed2.lastread);
    // }
  }
       if(PumpTimer.isTimerReady()){
          digitalWrite(PUMPPIN,LOW);
      }

//KEEP CURRENT SETTINGS TO COMPARE WITH NEW READINGS
    currTemp = tempF;
    currPresr = inHg;
    currHum = humidRH;
  }
    if(subValue ==1){
      
    }else{
     
    }

   // }
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
// void pumpWater(){
//    static unsigned int plast;
//    Serial.printf("plast = %i\n", plast);
//    Serial.printf("Function pumpWater called \n");
//    //digitalWrite(PUMPPIN,HIGH);

//     if ((millis()-plast)>500) {
//        // digitalWrite(PUMPPIN,LOW);
//         plast = millis();
        
//     }   
//    //digitalWrite(PUMPPIN,HIGH);
// }
// void ShowEnviro(){
//   //Serial.printf("T: %i\nP: %i\nH: %i\n",tempF,pressPA,humidRH);
      
    
    
// }