#include <math.h>
#include <Wire.h> 
#include "IntersemaBaro.h"


Intersema::BaroPressure_MS5607B baro(true);


#include <MMA_7455.h>           //Include the MMA_7455 library
MMA_7455 accel = MMA_7455();    // Make MMA7455 object
char xVal, yVal, zVal;          // Return value variables
double alt;
double prevAlt;

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
int Addr = 105;                 // I2C address of gyro
int x, y, z;

int zeroAlt = 0;
double aMag;


double g = 9.81; // m/s^2
double m = 10; // kg
double t_c;
double t_b;

int ledPin = 7;

int i = 0;
int w = 0;
int f = 0;
int n = 0;
int tick = 0;
float currentTime = 0;
float timeStep;

double velocity = 0;

void setup() { 
    
    Wire.begin();
    Serial.begin(9600);
    baro.init();
    
    accel.initSensitivity(2); // sensitivity to g-forces (2=2g, etc)
    
    writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
    writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
    writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
    delay(100);                   // Wait to synchronize 
    
    pinMode(ledPin, OUTPUT);
    
    Serial.println("");
    Serial.println("GETTING OUT OF BED");
    delay(200);
    Serial.println("");
    
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(50);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
       
    calibrate(); // calibrates all sensors given nonmoving vehicle on launchpad
}

void loop() {
  
  double previousTime = currentTime;
  currentTime = millis()/1000.0;
  timeStep = currentTime - previousTime;
  
  if (n < 6) {
    Serial.println("");
    Serial.print(tick); Serial.print("\t");
    tick++;
    Serial.print(currentTime); Serial.print("\t");
    Serial.print(timeStep * 1000); Serial.print("\t \t");
    
    integrateAcceleration();
    Serial.print(velocity); Serial.print("\t");
    
  }
  
  if (n == 0) {
    n = checkForLaunch(); // checks continuously for UPWARDS acceleration
    // returns n = 1 if LAUNCH has occured, n = 0 otherwise
  }
  else if (n == 1) {
    n = checkForBurnout(); // checks continuously for DOWNWARDS acceleration
    // returns n = 2 if BURNOUT has occured, n = 1 otherwise
  }
  else if (n == 2) {
    Serial.println("DEPLOY DRAG SYSTEM");
    Serial.print("Velocity is ");
    Serial.print(velocity);
    Serial.println(" m/s");
    n = 3;
  }
  else if (n == 3) {
    n = deployAndCalculate(); // deploys airbrakes, calculates t_c value given altitude and velocity
    // returns n = 4 once it has run ONLY ONE TIME -- DOES NOT RUN MORE THAN ONCE
  }
  else if (n == 4) {
    n = waitForRetract(); // waits t_c seconds, then retracts flaps
    // returns n = 5 if FLAP RETRACT has occured, n = 4 otherwise
  }
  else if (n == 5) {
    n = checkForApogee(); // continually checks for change in altitude
    // returns n = 6 if APOGEE IS REACHED, n = 5 otherwise
  }
  else if (n == 6) {
    Serial.print("GOOD JOB");
    n = 7;
  }
  
}

void calibrate () {
  
    Serial.println("CALIBRATING");
    Serial.println("");
    
    // start calibration routine
      
    zeroAlt = baro.getHeightCentiMeters();
    delay(300);
    zeroAlt = (zeroAlt + baro.getHeightCentiMeters())/2;
    delay(300);
    zeroAlt = (zeroAlt + baro.getHeightCentiMeters())/2;
    delay(300);
    zeroAlt = (zeroAlt + baro.getHeightCentiMeters())/2;
    
    xVal = accel.readAxis('x');   // Read X Axis
    yVal = accel.readAxis('y');   // Read Y Axis
    zVal = accel.readAxis('z');   // Read Z Axis
    accel.calibrateOffset(-xVal,-yVal,-(zVal+.75));

    // end calibration routine
    
}

void measureLoop () {
    // start altimeter routine
    int alt = baro.getHeightCentiMeters()-zeroAlt; // measures altitude
    // end altimeter routine
    
    // start accelerometer routine
    xVal = accel.readAxis('x');   // Read X Axis
    yVal = accel.readAxis('y');   // Read Y Axis
    zVal = accel.readAxis('z');   // Read Z Axis
    aMag = sqrt(square(xVal) + square(yVal) + square(zVal));
    // end accelerometer routine
    
    // start gyro routine
    getGyroValues();              // Get new values
    // end gyro routine
    
    // print everything to serial
    
    Serial.print((float)(alt)/100+1);
    Serial.print("\t \t");
  
    Serial.print(xVal, DEC); Serial.print("\t");
    Serial.print(yVal, DEC); Serial.print("\t");
    Serial.print(zVal, DEC); Serial.print("\t");
    Serial.print((int)aMag, DEC); Serial.print("\t");
    
    Serial.print("\t \t"); Serial.print(x / 114);    // In following Dividing by 114 reduces noise
    Serial.print("\t"); Serial.print(y / 114);
    Serial.print("\t"); Serial.print(z / 114);
}

int checkForLaunch () {
    
    // checks continuously for UPWARDS acceleration approx. every 20 ms
    // returns n = 1 if LAUNCH has occured, n = 0 otherwise
    
    zVal = accel.readAxis('z');   // Read Z Axis
  
    Serial.print(zVal, DEC); Serial.print("\t");
    
    if (zVal > 20) {
      i++;   }
    else {
      i = 1;  }
    
    if (i == 6) {
      Serial.println("\tLIFTOFF");
      digitalWrite(ledPin, HIGH);
      Serial.println("GOOD JOB! PUT ME DOWN.");
      delay(500);
      Serial.println("3");
      delay(500);
      Serial.println("2");
      delay(500);
      Serial.println("1");
      delay(500);
      digitalWrite(ledPin, LOW);
      return n = 1;
      i=1;  }
      
    for (int j = 0; j < i; j++) {
      Serial.print("|");
    }
    
    return n = 0;
}

int checkForBurnout() {
  
    // checks continuously for DOWNWARDS acceleration approx every 20 ms
    // returns n = 2 if BURNOUT has occured, n = 1 otherwise
  
    zVal = accel.readAxis('z');   // Read Z Axis
    
    Serial.print(zVal, DEC); Serial.print("\t");
    
    if (zVal < -30) {
      w++;   }
    else {
      w = 1;  }
    
    for (int j = 0; j < w; j++) {
      Serial.print("|");
    }
    
    if (w == 6) {
      Serial.println("BURNOUT CONFIRMED");
      t_b = currentTime;
      return n = 2;
      i=1;  }
    
    return n = 1;

}

int deployAndCalculate() {
  
    // deploys airbrakes, calculates t_c value given altitude and velocity
    // returns n = 4 once it has run ONLY ONE TIME -- DOES NOT RUN MORE THAN ONCE
    
    digitalWrite(ledPin, HIGH);
    
    t_c = t_b + 8; // placeholder for actual t_c calculation;
    
    Serial.print("FLAPS DEPLOYING NOW - FLAPS CLOSING AT t = ");
    Serial.print(t_c);
    Serial.print(" seconds - ");
    Serial.print(t_c - t_b);
    Serial.print(" seconds until RETRACT");
    return 4;
}

int waitForRetract() {
  
    // waits t_c seconds, then retracts flaps
    // returns n = 5 if FLAP RETRACT has occured, n = 4 otherwise
    
    if (currentTime >= t_c) {
      Serial.print("FLAPS CLOSING");
      digitalWrite(ledPin, LOW);
      return 5;
    }
    else if (tick % 20 == 1) {
      Serial.print("FLAPS OUT - ");
      Serial.print(t_c - currentTime);
      Serial.print(" seconds until RETRACT");
      return 4;
    }
    else {
      return 4;
    }
}

int checkForApogee() { 
  
    // continually checks for change in altitude
    // returns n = 6 if APOGEE IS REACHED, n = 5 otherwise
    
    prevAlt = alt;
    
    // start altimeter routine
    alt = baro.getHeightCentiMeters()-zeroAlt; // measures altitude
    // end altimeter routine
    
    double deltaAlt = alt - prevAlt;
    
    Serial.print(deltaAlt/100); Serial.print("\t\t");
    
    if (deltaAlt < -80) {
      f++;   }
    else {
      f = 1;  }
    
    for (int j = 0; j < f; j++) {
      Serial.print(":^)");
    }
    
    if (f == 3) {
      Serial.println("     DEPLOY THE PARACHUTES");
      return 6;
      digitalWrite(ledPin, HIGH);
      f=1;  }
    
    return 5;
}
  
void integrateAcceleration() {
  
   zVal = accel.readAxis('z');   // Read Z Axis
   if (abs(zVal) > 1) { // this filters out the garbage, hopefully
     velocity = velocity + zVal * (g/63) * timeStep;
   }
   
}

void getGyroValues () {
  byte MSB, LSB;

  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  x = ((MSB << 8) | LSB);

  MSB = readI2C(0x2B);
  LSB = readI2C(0x2A);
  y = ((MSB << 8) | LSB);

  MSB = readI2C(0x2D);
  LSB = readI2C(0x2C);
  z = ((MSB << 8) | LSB);
}

int readI2C (byte regAddr) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);                // Register address to read
    Wire.endTransmission();             // Terminate request
    Wire.requestFrom(Addr, 1);          // Read a byte
    while(!Wire.available()) { };       // Wait for receipt
    return(Wire.read());                // Get result
}

void writeI2C (byte regAddr, byte val) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);
    Wire.write(val);
    Wire.endTransmission();
}
