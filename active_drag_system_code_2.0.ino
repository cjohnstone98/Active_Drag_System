// Current Issues:
// 

// SOURCE ALL NECESSARY LIBRARIES
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_LSM9DS0.h>
#include <Wire.h>
#include "Arduino.h"


//GLOBALLY DEFINE VARIABLES AND SENSORS
// all-scope variable declaration
int currentGyroRateX;
int currentGyroRateY;
// long stepSize = 10;   // long format required by .runToPosition(...) function
int gyroScale = 245;
long stepHome = 0; // minimum rotation distance before motor burnout (home)
long maxStep = 109; // maximum rotation distance before motor burnout, adjust later to keep fins from breaking
int currentGyroRate;
float altitudeNM = 82946; /* [pa] subtract this from the altitude reading at any given time */
float gyroXStart;
float gyroYStart;
long angleScalingFactor = 55 // steps/deg
long angularSpeedScalingFactor = 55 // steps/deg

// Initialize BMP388
Adafruit_BMP3XX bmp;  // i2c sensor

// Initialize LSM9DS0
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();  // i2c sensor
void setupSensor() {
   // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
 
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/* stepper motor setup */
  Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
  Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers

// On the top shield, connect two steppers, each with 513 steps
  Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(513, 1);
  Adafruit_StepperMotor *myStepper3 = AFMStop.getStepper(513, 2);

    // On the bottom shield connect two steppers, each with 513 steps
  Adafruit_StepperMotor *myStepper2 = AFMSbot.getStepper(513, 1);
  Adafruit_StepperMotor *myStepper4 = AFMSbot.getStepper(513, 2);
    void forwardstep1() { 
      myStepper1->onestep(FORWARD, DOUBLE);   // you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
      }
    void backwardstep1() { 
      myStepper1->onestep(BACKWARD, DOUBLE);
      }
    void forwardstep2() { 
      myStepper2->onestep(BACKWARD, DOUBLE);
      }
    void backwardstep2() { 
      myStepper2->onestep(FORWARD, DOUBLE); 
      }
      void forwardstep3() { 
      myStepper3->onestep(BACKWARD, DOUBLE);
      }
    void backwardstep3() { 
      myStepper3->onestep(FORWARD, DOUBLE);
      }
      void forwardstep4() { 
      myStepper4->onestep(BACKWARD, DOUBLE);
      }
    void backwardstep4() { 
      myStepper4->onestep(FORWARD, DOUBLE);
      }
    AccelStepper stepper1(forwardstep1, backwardstep1);
    AccelStepper stepper2(forwardstep2, backwardstep2);
    AccelStepper stepper3(forwardstep3, backwardstep3);
    AccelStepper stepper4(forwardstep4, backwardstep4);
  
void setup() {    // put your setup code here, to run once: 

while (!Serial);
Serial.begin(9600);
 /* Stepper Motor Setup */
  AFMStop.begin();        // Start the top shield
  AFMSbot.begin();        // Start the bottom shield
    ///*
    stepper1.setSpeed(25);
    stepper1.setAcceleration(500);
      stepper1.move(50);
      stepper1.runToPosition();
      stepper1.move(-50);
      stepper1.runToPosition();
    //*/
    ///*
    stepper2.setSpeed(25);
    stepper2.setAcceleration(500);
      stepper2.move(50);
      stepper2.runToPosition();
      stepper2.move(-50);
      stepper2.runToPosition();
    //*/
    ///*
    stepper3.setSpeed(25);
    stepper3.setAcceleration(500);
      stepper3.move(50);
      stepper3.runToPosition();
      stepper3.move(-50);
      stepper3.runToPosition();
    //*/
    ///*
    stepper4.setSpeed(25);
    stepper4.setAcceleration(500);
      stepper4.move(50);
      stepper4.runToPosition();
      stepper4.move(-50);
      stepper4.runToPosition();
    //*/
    // sets anchored zero positions for the motors
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);

  /* BMP388 Setup Stuff */
    Serial.println("Checking for BMP");
    if (!bmp.begin()) {  
      Serial.println("Could not find a valid BMP388 sensor, check wiring!");
      while (1);
    }
    Serial.println("Found BMP388...checking for LSM9DS0");
      // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
 /* gyroscope sensor setup */
  if (!lsm.begin()) {
    Serial.println("Ooops, no LSM9DS0 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  delay(250);
  Serial.println("...");
  delay(250);
  Serial.println("...");
  delay(250);
  Serial.println("Beginning Operation");

// zeroes the gyroscope based on start orientation
      lsm.readGyro();
      gyroXStart = lsm.gyroData.x/gyroScale;
      gyroYStart = lsm.gyroData.y/gyroScale;
      /* subtract the start values from each reading to "zero it" before launch */
      gyroCurrentXAngle = 0;
      gyroCurrentYAngle = 0;
}     // end setup


  float currentAltitude() {
      float sensorAltitude1 = bmp.readPressure() / 1000; /* gets the pressure reading from the sensor */
      float currentAltitude = sensorAltitude1 - altitudeNM;  /* yeilds the actual altitude */
      Serial.println("Approximate Altitude:  ");
      Serial.println(currentAltitude);
      return currentAltitude;
        }
        
    // NEED TO KNOW THE UNITS OF THE ALTITUDE
    float altitudeRate() {
      float currentAlt1 = currentAltitude();
      delay(50);
      float currentAlt2 = currentAltitude();
      float altRate = (currentAlt2 - currentAlt1)/.05;
      Serial.println("Approximate Altitude Rate Of Change:  ");
      Serial.println(altRate);
      return altRate;
        }
        
  sensors_event_t accel, mag, gyro, temp;
  
  double gyroStateX(gyroCurrentXAngle) {      // yields the difference in rate of change for rotation around the x-axis
     gyroPreviousXAngle = gyroCurrentXAngle;
     lsm.readGyro();
        double gyroStateX1 = lsm.gyroData.x/gyroScale;
        delay(50);       // too long and it reacts too slowly to be good and flight, too quick and the measurements are the same, returning zero.
     lsm.readGyro();
        double gyroStateX2 = lsm.gyroData.x/gyroScale;
     // improve the integral by shortening the time delay and increasing the number of readings
          // average angular velocity 
        double gyroStateRateX = (gyroStateX2 + gyroStateX1)/2;
          // take a two step integral of the rates over the time step to find the change in angle
        double gyroCurrentXAngle = (gyroStateX2 - gyroStateX1) * 0.05 + gyroPreviousXAngle;
        return gyroStateRateX, gyroCurrentXAngle;
          } 

  double gyroStateY(gyroCurrentYAngle) {       // yields the difference in rate of change for rotation around the y-axis
    gyroPreviousYAngle = currentYAngle;
    lsm.readGyro();
      double gyroStateY1 = lsm.gyroData.y/gyroScale;
      delay(50);
    lsm.readGyro();
    // delta-v * delta-t + theta-1 = theta-2 = current angle
      double gyroStateY2 = lsm.gyroData.y/gyroScale;
      double gyroStateRateY = (gyroStateY2 + gyroStateY1)/2;
      double gyroCurrentYAngle = (gyroStateY2 - gyroStateY1) * 0.05 + gyroPreviousYAngle;
      return gyroStateRateY, gyroCurrentYAngle;
  }


// put your main code here, to run repeatedly:
void loop() {
  lsm.getEvent(&accel, &mag, &gyro);
  // probably want accel x,y,z and gyro x,y,z to run conditionals here
  // reads the pressure data from the sensor
  int8_t bmp3_get_sensor_data(uint8_t 2, struct bmp3_data *data, struct bmp3_dev *dev);
  int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp3_dev *dev);

 long altRate = altitudeRate();
 // long currentAlt = currentAltitude();
 long currentAlt = 0; // REMOVE AFTER TESTING IS COMPLETED

/* Begin during-flight while loop circuit */
/* while the rocket is in flight up to apogee AND while not descending, actuate the ADS | MASTER LOOP*/
  while (/*(currentAlt > 1000) &&*/ (currentAlt < 30000) /*&& (altRate > 0)*/) {
          
      Serial.println("Approximate Current Altitude (m) & Altitude Rate of Change (m/s):  ");
      Serial.println(currentAlt);
      Serial.println(altRate);
      
      // reduce the delay to 50 for actual flight
      delay(500);
      
      lsm.readGyro();

// NEED TO DETERMINE AN APPROPRIATE ANGLE SCALING FACTOR THAT ACCOUNTS FOR CURRENT ANGLE AND RATE OF ANGLE CHANGE
      
      currentGyroStateX = gyroStateX(gyroCurrentXAngle);     
         gyroStateRateX = currentGyroStateX(1);             // gets current "rate" of tilting around X-Axis
         gyroCurrentXAngle = currentGyroStateX(2);          // gets current angle of tilting around X-Axis
      currentGyroStateY = gyroStateY(gyroCurrentYAngle);
        gyroStateRateY = currentGyroStateY(1);
        gyroCurrentYAngle = currentGyroStateY(2);
      // heavily check this logic with system, there is a lot of uncertainty
      motor1Position = round(gyroCurrentXAngle * angleScalingFactor + gyroStateRateX * angleSpeedScalingFactor);
        if (motor1Position > maxStep) {
          motor1Position = maxStep;
        }
        // -1 is used because of uncertainty, fine tune but using 0 as baseline is probably unrealistic
        else if (motor1Position < -1) {
          motor3Position = -motor1Position;
          motor1Position = 0; 
            if (motor3Position > maxStep) {
              motor3Position = maxStep;
            }
        }
        else if (motor1Position > -1 || motor1Position < 1) {
          motor1Position = 0;
          motor3Posiiton = 0;
        }
      motor2Position = round(gyroCurrentYAngle * angleScalingFactor + gyroStateRateY * angleSpeedScalingFactor);
        if (motor2Position > maxStep) {
          motor2Position = maxStep;
        }
        else if (motor2Position < -1) {
          motor4Position = -motor2Position;
          motor2Position = 0;
            if (motor4Position > maxStep) {
              motor4Position = maxStep;
            }
        }
        else if (motor2Position > -1 || motor2Position < 1) {
          motor2Position = 0;
          motor4Posiiton = 0;
        }
        
      stepper1.run();
      stepper2.run();
      stepper3.run();
      stepper4.run();
      
        stepper1.move(motor1Position);
        stepper1.runToPosition();
        Serial.println("Current Motor 1 Step:  ");
        Serial.println(stepper1.currentPosition());
        if (!stepper1.currentPosition() == 0) {
          gyroCurrentXAngle = stepper1.currentPosition()/angleScalingFactor;
        }
        
        stepper2.move(motor2Position);
        stepper2.runToPosition();
        Serial.println("Current Motor 2 Step:  ");
        Serial.println(stepper2.currentPosition());
        if (!stepper2.currentPosition() == 0) {
          gyroCurrentYAngle = stepper2.currentPosition()/angleScalingFactor;
        }
        
        stepper3.move(motor3Position);
        stepper3.runToPosition();
        Serial.println("Current Motor 3 Step:  ");
        Serial.println(stepper3.currentPosition());
        if (!stepper3.currentPosition() == 0) {
          gyroCurrentXAngle = -stepper3.currentPosition()/angleScalingFactor;  // NEGATIVE OF THE POSITION SO A NEGATIVE ANGLE CORRESPONDS TO MOTOR 3 ACTUATION
        }

        stepper4.move(motor4Position);
        stepper4.runToPosition();
        Serial.println("Current Motor 4 Step:  ");
        Serial.println(stepper4.currentPosition());
        if (!stepper4.currentPosition() == 0) {
          gyroCurrentYAngle = -stepper4.currentPosition()/angleScalingFactor;  // NEGATIVE OF THE POSITION SO A NEGATIVE ANGLE CORRESPONDS TO MOTOR 4 ACTUATION
        }
      
 // long currentAlt = currentAltitude(); // checks altitude again 
 // long altRate = altitudeRate();
}

}

