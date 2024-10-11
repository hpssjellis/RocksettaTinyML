/*
 * webSerial for testing javascript connection with an arduino
 * 
 * Latest work at   https://github.com/hpssjellis/tinyMLjs
 * 
 * 
 */



#include <Arduino.h> // Only needed for https://platformio.org/


// REV 2 CHANGES HERE 

/* For LSM9DS1 9-axis IMU sensor */
//#include <Arduino_LSM9DS1.h>         // REV 1
#include "Arduino_BMI270_BMM150.h" // REV 2

/* For HTS221 Temperature and humidity sensor */
// not needed for this webSerial code
//#include <Arduino_HTS221.h>          // REV 1
//#include <Arduino_HS300x.h>        // REV 2
//#define HTS HS300x                   // For REV 2 to work with old REV1 code


#define FREQUENCY_HZ        36     // how many samples per second 
#define COLLECTION_SECONDS  1     // how many seconds to collect samples
#define INTERVAL_MS  (1000 / (FREQUENCY_HZ + 1)) // need for the timer 
#define CONVERT_G_TO_MS2    9.80665f   // accleration conversion

int myMaxData = FREQUENCY_HZ * COLLECTION_SECONDS;
int myCount = 0;
int myDelay = INTERVAL_MS;   // non-block delay in milliseconds
unsigned long myStart; 

String readString;
bool mySendData = true;


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // onboard LED, HIGH = off
  //while (!Serial) {}                 // do nothing and wait
  
  myStart = millis();   // set delay
  randomSeed(analogRead(A0));  // AO pin on XIAO does not have to be connected to anything
  Serial.println("accX,accY,accZ");  // CSV file heading titles
  
  if (!IMU.begin()) {
     Serial.println("Failed to initialize IMU!");
     while (1);
  }
}

void loop() {
  float x, y, z;
  if ( (millis() - myStart ) >= myDelay) {       
     myStart = millis();      //  reset the delay time
     myCount += 1;
     IMU.readAcceleration(x, y, z);
     // convert raw acceleration to acceleration due to gravity
     x *= CONVERT_G_TO_MS2;
     y *= CONVERT_G_TO_MS2;
     z *= CONVERT_G_TO_MS2;
     //Serial.println( String(myStart)+ "," + String(analogRead(A0)) + "," + String(analogRead(A1)) + "," +  String(analogRead(A2))  );
     if (myCount >= myMaxData){
        mySendData = false;   // stop sending data when amount reached
     }
     if (mySendData){
       // Serial.println( String(x) + "," + String(y) + "," +  String(z)  );
        Serial.println( String(x) + "," + String(y) + "," +  String(z) );
     } 
   }

   while (Serial.available()) {
    delay(3);  
    char myChar = Serial.read();
    readString += myChar; 
  }

  if (readString.length() > 0) {
    readString.trim();  // get rid of last weird character
    if (readString == "a"){
      digitalWrite(LED_BUILTIN, LOW); //onboard LED LOW = on
    }
    if (readString == "b"){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (readString == "start"){
      mySendData = true;
      myStart = millis();      //  reset the delay time
      myCount = 0;
    }
    if (readString == "stop"){
      mySendData = false;
      Serial.println("Stopping at count: "+ String(myCount));  // CSV file heading titles
    }
    if (readString == "firstline"){
      Serial.println("accX,accY,accZ");  // CSV file heading titles
    }
    readString="";
  } 
  
}
   
  
