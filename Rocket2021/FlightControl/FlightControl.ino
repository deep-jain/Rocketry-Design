#include <SD.h>                     // SD card library
#include <Adafruit_BMP085_U.h>      // for BMP085 altimeter
#include <Adafruit_LSM303_U.h>      // for LSM303 accelerometer

#include <RocketMath.h>             // for Kalman filter, trajectory, and flaps
#include "Vehicle.h"                // for rocket vehicle characteristics

#define CLR(x,y) (x&=(~(1<<y)))     // macros for writing to registers
#define SET(x,y) (x|=(1<<y))

/* PIN DEFINITIONS */
#define RED_PIN                     3 // turns on if fatal error occurs
#define BLUE_PIN /* PIN 4 BROKEN */ 5 // blinks with every tick
#define GREEN_PIN                   5 // no purpose yet
// #define CONTACT_A                   6 // contact switch A
// #define CONTACT_B                   7 // contact switch B
#define FLAP_POSITIVE               8 // flap motor positive terminal
#define FLAP_NEGATIVE               9 // flap motor negative terminal

/* FLIGHT STATE DEFINITIONS */
#define ON_LAUNCHPAD                0 // indicates rocket is on the launchpad. accel = 0.
#define MOTOR_BURN                  1 // indicates motor is burning. accel > 0.
#define APOGEE_COAST                2 // indicates motor has burned out. accel < 0.
#define POST_APOGEE                 3 // indicates vehicle is falling

#define NOMINAL_DT                  0.05 // seconds

double LAUNCHPAD_ALT;               // altitude of launchsite
double REST_ACCEL;                  // acceleration of non-accelerating vehicle
long int BEGIN_TIME;                // time the main loop begins
uint8_t stage = ON_LAUNCHPAD;       // current state of the vehicle

Adafruit_LSM303_Accel_Unified       lsm = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_BMP085_Unified             bmp = Adafruit_BMP085_Unified(10085);

KalmanFilter filter(0, 0.01);
File sensorData;
FlapInterface flaps(FLAP_POSITIVE, FLAP_NEGATIVE, 50);

/*
 *      __
 *  ___( o)>   i am the wise duck of code, your code will compile
 *  \ <_. )    without errors, but only if you say "compile well ducko"
 *   `---'
 */

void setup()
{
    // set the pinmode for the diagnostic LEDs
    pinMode(RED_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);

    // flash the LEDs and flaps to ensure they're all working
    for (uint8_t i = 0; i < FLIGHT_NUMBER + 1; i++) // twice if second flight
    {
        digitalWrite(RED_PIN, HIGH);
        digitalWrite(BLUE_PIN, HIGH);
        digitalWrite(GREEN_PIN, HIGH);
        flaps.deploy();
        delay(1000);
        digitalWrite(RED_PIN, LOW);
        digitalWrite(BLUE_PIN, LOW);
        digitalWrite(GREEN_PIN, LOW);
        flaps.retract();
        delay(1200);
    }

    // initialize the LSM303, the BMP085, and the SD card
    if(!lsm.begin()) fatal_error(1);
    if(!bmp.begin()) fatal_error(2);
    if(!SD.begin()) fatal_error(3);

    // initialize the log file
    char filename[] = "LOG000.csv";
    for (uint8_t i = 0; i < 1000; i++)
    {
        filename[3] = i/100 + '0';
        filename[4] = (i/10)%10 + '0';
        filename[5] = i%10 + '0';
        if (!SD.exists(filename))
        {
            sensorData = SD.open(filename, FILE_WRITE);
            break;
        }
    }
    if (!sensorData) fatal_error(4);

    LAUNCHPAD_ALT = getAltitude(100);
    REST_ACCEL = getAcceleration(100);

    // initialize the Kalman filter
    // state transition matrix, F
    filter.F[0][0] = 1;
    filter.F[1][1] = 1;
    // sensor covariance matrix, R
    filter.R[0][0] = 0.02;
    filter.R[1][1] = 0.001;
    // state matrix, X
    filter.X[0][0] = 0; // altitude
    filter.X[1][0] = 0; // acceleration
    // uncertainty matrix, P
    filter.P[0][0] = 10;
    filter.P[1][1] = 10;

    sensorData.println(filename);
    sensorData.println("WATCH OUT FOR THOSE WRIST ROCKETS");
    sensorData.println("Notes:");
    sensorData.print("Nominal dt: ");
    sensorData.println(NOMINAL_DT);
    sensorData.print("Launchsite altitude: ");
    sensorData.println(LAUNCHPAD_ALT);
    sensorData.print("Resting acceleration: ");
    sensorData.println(REST_ACCEL);
    sensorData.println("time,raw alt,k alt,raw accel,k accel,vel,s1,s2");

    // flash again to indicate setup finished
    digitalWrite(BLUE_PIN, HIGH);
    flaps.deploy();
    delay(1000);
    digitalWrite(BLUE_PIN, LOW);
    flaps.retract();
    delay(2000);
    flaps.kill();

    BEGIN_TIME = millis();
    delay(NOMINAL_DT * 1000);
}

void loop()
{
    bool FLAP_STATE = false;
    static double alt_prev;
    double current_time, dt, raw_altitude, altitude, raw_accel, accel;
    update_time(&current_time, &dt);
    raw_altitude = getAltitude(1) - LAUNCHPAD_ALT;
    raw_accel = getAcceleration(1); // - REST_ACCEL;

    // filter wizardry to clean up alt and accel data
    filter.F[0][1] = dt * dt; // relation between acceleration and altitude
    float Z[MEAS] = {raw_altitude, raw_accel};
    float* X = filter.step((float*) Z);
    altitude = X[0];
    accel = X[1];
    double velocity = (altitude - alt_prev)/dt;

     // status indicators allow internal state to be recorded efficiently
    uint8_t major_status = 100;
    uint8_t minor_status = 0;
    // stage-specific progression logic
    const uint8_t ticksToAdvance = 6;
    if (stage == ON_LAUNCHPAD)
    {
        static uint8_t high_vel_count;
        if (velocity > 3) high_vel_count++;
        else if (high_vel_count > 0) high_vel_count--;
        if (high_vel_count > 2 * ticksToAdvance) stage++;

        major_status = ON_LAUNCHPAD;
        minor_status = high_vel_count;
    }
    if (stage == MOTOR_BURN)
    {
        static uint8_t low_accel_count;
        if (accel < -10) low_accel_count++;
        else if (low_accel_count > 0) low_accel_count--;
        if (low_accel_count > ticksToAdvance) stage++;

        major_status = MOTOR_BURN;
        minor_status = low_accel_count;
    }
    if (stage == APOGEE_COAST)
    {
        /*
         * alt_next and vel_next are the predicted state of the rocket in
         * dt seconds, assuming the flaps are deployed, and are found by
         * solving the following differential equation:
         *
         *     dv/dt + (kv^2 / m) + g = 0
         *
         * This yields the model found in Equations.h.
         * ta_predict is the time to apogee which is predicted for the
         * next step, and apo_predict is the apogee altitude,
         * assuming the flaps are retracted for the rest of the flight.
         * As long as this predicted altitude is greater than the target,
         * it is desirable to lower it for this timestep, so the flaps
         * are opened. Otherwise, opening the flaps this step will cause the
         * vehicle to brake too hard, so the flaps are closed.
         */

        const uint8_t vmin = 15; // minimum velocity for flap control

        major_status = APOGEE_COAST;

        // predictive calculations determine where vehicle will be next step
        double alt_next = trajectory::alt(altitude, velocity, DRY_MASS, K_ACTIVE, dt);
        double vel_next = trajectory::vel(velocity, DRY_MASS, K_ACTIVE, dt);

        // describe the trajectory the vehicle will be on during next step
        double ta_predict = trajectory::t_a(vel_next, DRY_MASS, K_PASSIVE);
        double apo_predict = trajectory::alt(alt_next, vel_next, DRY_MASS, K_PASSIVE, ta_predict);

        // if the predicted altitude is acceptable, engage the flaps
        if (apo_predict > TARGET_ALT && vel_next > vmin)
        {
            FLAP_STATE = true;
        }
        else // otherwise, retract the flaps
        {
            FLAP_STATE = false;
        }

        static uint8_t low_vel_count;
        if (velocity < vmin)
        {
            low_vel_count++;
            FLAP_STATE = false;
        }
        else if (low_vel_count > 0) low_vel_count--;
        if (low_vel_count > ticksToAdvance) stage++;
        minor_status = FLAP_STATE;
    }
    if (FLAP_STATE && FLIGHT_NUMBER)
    {
        flaps.deploy();
    }
    else
    {
        flaps.retract();
    }

    alt_prev = altitude; // save previous altitude for faux derivative

    // write to log file
    sensorData.print(current_time, 4);
    sensorData.print(",");
    sensorData.print(raw_altitude);
    sensorData.print(",");
    sensorData.print(altitude);
    sensorData.print(",");
    sensorData.print(raw_accel);
    sensorData.print(",");
    sensorData.print(accel);
    sensorData.print(",");
    sensorData.print(velocity);
    sensorData.print(",");
    sensorData.print(major_status);
    sensorData.print(",");
    sensorData.println(minor_status);

    static uint8_t flush;
    flush++;
    if (flush == 50)
    {
        SET(PORTD, BLUE_PIN);
        sensorData.flush();
        flush = 0;
        CLR(PORTD, BLUE_PIN);
    }

    // wait for next tick if computations were fast enough
    double compTime = (double) (millis() - BEGIN_TIME)/1000 - current_time;
    if (compTime < NOMINAL_DT)
    {
        delay((NOMINAL_DT - compTime)*995);
    }
}

void update_time(double* current_time, double* dt)
{
    // get current time and time since last call
    static double lastTime;
    *current_time = (double) (millis() - BEGIN_TIME)/1000;
    *dt = *current_time - lastTime;
    lastTime = *current_time;
}

void fatal_error(uint8_t error)
{
    while(1)
    {
        for (uint8_t i = 0; i < error; i++)
        {
            digitalWrite(RED_PIN, HIGH);
            delay(250);
            digitalWrite(RED_PIN, LOW);
            delay(250);
        }
        delay(500);
    }
}

double getAcceleration(uint8_t measurements)
{
    sensors_event_t event;
    double sum = 0;
    for (uint8_t i = 0; i < measurements; i++)
    {
        lsm.getEvent(&event);
        sum += event.acceleration.x;
    }
    return sum/measurements;
}

double getAltitude(uint8_t measurements)
{
    sensors_event_t event;
    double sum = 0;
    for (uint8_t i = 0; i < measurements; i++)
    {
        bmp.getEvent(&event);
        sum += bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
            event.pressure);
    }
    return sum/measurements;
}
