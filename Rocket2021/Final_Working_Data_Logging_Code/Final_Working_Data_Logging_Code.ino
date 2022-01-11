/*
This code was created by Aaron Brown.
It used a few different sensor API provided by the following URL's,
https://github.com/adafruit/Adafruit_Sensor/archive/master.zip
https://github.com/adafruit/Adafruit_BMP085_Unified/archive/master.zip
https://github.com/adafruit/Adafruit_LSM303DLHC/archive/master.zip
https://github.com/adafruit/Adafruit_L3GD20_U/archive/master.zip
https://github.com/adafruit/Adafruit_9DOF/archive/master.zip

Not all of this code was written by me, I have just taken from the examples, and modified it.

*/
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
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);


long interval = 20;
long previousmillis= 0;
File mySensorData;

void initSensors() {
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin()) {
  /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
    
  }
  /*if(!GPS.begin() ){
     Serial.print("Ooops, no GPS detected ... Check your wiring or shield!");
     while(1);
  }*/
}


void setup(){

  
  Serial.begin(115200); //I need help for choosing this correctly
  sensors_event_t event;
  gyro.enableAutoRange(true); /* Enable auto-ranging */
  initSensors();  /* Initialise the sensors */

  if (!SD.begin(10))     // if you're using an UNO, you can use this line instead
      Serial.println("Card init. failed!"); /*initialize SD card dreader*/

 
 
  mySensorData = SD.open("FuckingShit.txt", FILE_WRITE);
  mySensorData.print("Temperature(C)");
  mySensorData.print("\t");
  mySensorData.print("Pressure(hPa)");
  mySensorData.print("\t");
  mySensorData.print("Altitude(m)");
  mySensorData.print("\t");
  mySensorData.print("Roll(deg)");
  mySensorData.print("\t");
  mySensorData.print("Pitch(deg)");
  mySensorData.print("\t");
  mySensorData.print("Heading(deg)");
  mySensorData.println();

  mySensorData.close();

  
  
}
void loop(){


        unsigned long currentmillis = millis();
        if(currentmillis - previousmillis > interval){
              previousmillis = currentmillis;
              //Setting up sensor events
              sensors_event_t accel_event; //For accelerometer
              sensors_event_t mag_event; //For magnetometer
              sensors_vec_t   orientation; //for orientation
              sensors_event_t event;  //For pressure and temperature
              accel.getEvent(&accel_event);
              mag.getEvent(&mag_event);
              bmp.getEvent(&event);  
              //Adafruit_GPS    GPS(&mySerial);
              
              float temperature;
              bmp.getTemperature(&temperature);
            
              
              float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
              float Altitude = bmp.pressureToAltitude(seaLevelPressure,event.pressure);
        
              if(mySensorData){
                    mySensorData = SD.open("FuckingShit.txt", FILE_WRITE);
                    mySensorData.print(temperature);                             //write temperature data to card
                    mySensorData.print("\t\t");                                                              //write a commma
                    mySensorData.print(event.pressure);                        //write pressure and end the line (println)
                    mySensorData.print("\t\t"); 
                      
                    mySensorData.print(Altitude);                             //write temperature data to card
                    mySensorData.print("\t\t");  

                    Serial.print(Altitude);
                    //Saving orientation roll, pitch, and heading
                   if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
                   {
                      float roll = orientation.roll;
                      float pitch = orientation.pitch;
                      float heading = orientation.heading;
                      
                      mySensorData.print(roll);                             //write temperature data to card
                      mySensorData.print("\t\t");  
                      mySensorData.print(pitch);                             //write temperature data to card
                      mySensorData.print("\t\t");  
                      mySensorData.print(heading);                             //write temperature data to card  
               
                   }
                  
              }
              
              /***********************************COMMENTED GPS CODE********************************************************/
              /*
              GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
              GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
              GPS.sendCommand(PGCMD_NOANTENNA);
              if (GPS.newNMEAreceived()) {
                char *GPS_sentence = GPS.lastNMEA();
                if (!GPS.parse(GPS_sentence))   // this also sets the newNMEAreceived() flag to false
                    return;
                uint8_t stringsize = strlen(GPS_sentence);
                if (stringsize != mySensorData.write((uint8_t *)GPS_sentence, stringsize))    //write the string to the SD file
                    Serial.print("Error baby!!");
                 strstr(GPS_sentence, "RMC") || strstr(GPS_sentence, "GGA");
              }
              */
               /*************************************************************************************************************/
        }
        mySensorData.println();
       mySensorData.close();
}
