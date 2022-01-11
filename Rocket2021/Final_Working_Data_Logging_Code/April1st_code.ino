#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_GPS.h>
#include <Adafruit_L3GD20_U.h>
#include <SD.h> //Load SD card library
#include<SPI.h> //Load SPI Library

//Setting up ID's for sensors
#include "Wire.h"    // imports the wire library for talking over I2C
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


const int cs = 4;


void initSensors() {
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  
  if(!bmp.begin()){
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
    
  }
  
}

void setup(){

   Serial.begin(9600); //I need help for choosing this correctly
  initSensors();  /* Initialise the sensors */


  Serial.print("Display Something!!!");
  pinMode(cs, OUTPUT);

  if (!SD.begin(cs))     // if you're using an UNO, you can use this line instead
      Serial.println("Card init. failed!"); /*initialize SD card dreader*/

  File mySensorData = SD.open("RocketData.csv", FILE_WRITE);
  
  if(mySensorData){

      Serial.print("Printing header!!!");
      mySensorData.print("Altitude(m)");
      mySensorData.print("\t");
      mySensorData.print("AccelX(m/s^2)");
      mySensorData.print("\t");
      mySensorData.print("AccelY(m/s^2)");
      mySensorData.print("\t");
      mySensorData.print("AccelZ(m/s^2)");
      mySensorData.print("\t");
      mySensorData.close();
  }
  
  

}

void loop(){
  
  sensors_event_t event;            //Object for pressure and altitude - FOR BMP085
  sensors_event_t accel_event;      //Object for accelerometer - for accelation - 9DOF
  sensors_vec_t   orientation;      //for X,Y,Z values of acceleration 
  

  String dataString = "";
  float altitude;
  float slp;
  float accelX;
  float accelY;
  float accelZ;
  

  if(bmp.getEvent(&event)){
    slp = SENSORS_PRESSURE_SEALEVELHPA;
    altitude = bmp.pressureToAltitude(slp, event.pressure);
  }else{
    Serial.print("Failed to get event");  
  }

  if(accel.getEvent(&accel_event)){
     accelX = accel_event.acceleration.x;  
     accelY = accel_event.acceleration.y;
     accelZ = accel_event.acceleration.z;
  }

  dataString = String(altitude) + "     " + String(accelX) + "     " + String(accelY) + "     " + String(accelZ);
  File mySensorData = SD.open("RocketData.csv", FILE_WRITE);
  
  if(mySensorData){
    mySensorData.print(dataString);
    Serial.print(dataString);
    mySensorData.close();
  }else{
    Serial.print("Failed to write to the file");
  }


  delay(20);
}
