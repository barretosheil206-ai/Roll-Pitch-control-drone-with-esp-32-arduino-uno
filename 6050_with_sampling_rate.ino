#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//******Setting up variables for the imu******:

float Ax;
float Ay;
float Az;
float T;

float Gx;
float Gy;
float Gz;

//Callibration of Gyro:
float GyroXOffset = 0;
float GyroYOffset = 0;
float GyroZOffset = 0;

//Gyro values:
float rollG = 0;
float pitchG = 0;
float yawG = 0;

//Callibration of Accelerometer:
float xMax = 1.07;
float xMin = -0.94;
float yMax = 0.99;
float yMin = -1.03;
float zMax = 0.91;
float zMin = -1.15;

float xOffset = (xMax+xMin)/2.;
float yOffset = (yMax+yMin)/2.;
float zOffset = (zMax+zMin)/2.;

float xScale = 2/(xMax-xMin);
float yScale = 2/(yMax-yMin);
float zScale = 2/(zMax-zMin);

//Accelerometer values:
float rollRaw = 0;
float pitchRaw = 0;
float yawRaw = 0;

//Complimentrary Values:
float rollComp = 0;
float pitchComp = 0;

float deltaRollG = 0;  //these are the increments for the gyro angle values
float deltaPitchG = 0;

//Complimentry fillter coefficients:
float Alpha = 0.20;
float Beta = (1-Alpha);

//IMU times:
unsigned long tStart = 0;

Adafruit_MPU6050 mpu;

//******Setting up time variables for the motor control******:

//time measurements during enterity of flight to check controller runs at 25Hz
unsigned long start_time = 0;            
unsigned long current_time = 0;
unsigned long old_time = 0;

//Setting the delay for the main loop to get 100Hz sampling:
int interval = 10;                  //0.01 secs
unsigned long dt = 0;          //actual time between samples


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //Set the mpu:
  mpu.begin();
  Serial.println("MPU6050 Started!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); //Needed for the accelerometers only
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);      //Needed for the gyros only, Note: rotating this faster will cause error
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  CallibrateGyro();

  start_time = millis();
  tStart = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = millis() - start_time; //find sampling rate time variables
  dt = current_time - old_time;

  //Get_IMU_Data();  //Get imu orientation

  //Serial.print("RollComp:");
  //Serial.print(rollComp);

  if (dt >= interval) {//check sampling rate
    
    Get_IMU_Data();  //Get imu orientation

    Serial.print("Time:");
    Serial.print(current_time/1000.0f);
    Serial.print(",");    
    Serial.print("RollComp:");
    Serial.print(rollComp);
    Serial.print(",");
    Serial.print("PitchComp:");
    Serial.print(pitchComp);
    Serial.print(",");
    Serial.print("YawG:");  //Yaw cant use the complimentry filter since we dont have its corresponding accelerometer values
    Serial.print(yawG);
    // Serial.print(",");
    // Serial.print("Yaw:");
    // Serial.print(yaw);
    Serial.print(",");
    Serial.print("UL:");
    Serial.print(90);
    Serial.print(",");
    Serial.print("LM:");
    Serial.println(-90);
    
    old_time = current_time;
  }

}

void CallibrateGyro() {
  Serial.println("Callibrating the Gyro, keep completely stationary...");
  delay(1000);
  int i;

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  int numIterations = 100;

  for (i = 0; i<numIterations; i = i + 1) {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  //this makes the measurenment
    sumX = sumX + g.gyro.x;
    sumY = sumY + g.gyro.y;
    sumZ = sumZ + g.gyro.z;

    delay(10);
  }

  GyroXOffset = sumX/numIterations;
  GyroYOffset = sumY/numIterations;
  GyroZOffset = sumZ/numIterations;
}

void Get_IMU_Data() {
  
  //find imu timings:
  static unsigned long last_time = 0;  //this line only runs once and never again so last_time doesn't get updated to zero again
  unsigned long now = millis();
  float delta_time = now - last_time;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  //this makes the measurenment

  //**********Define accelerometer values**********:
  Ax = a.acceleration.x/9.81;  //this gets the corresponding component out from a
  Ay = a.acceleration.y/9.81;
  Az = a.acceleration.z/9.81;

  Ax = xScale*(Ax-xOffset);
  Ay = yScale*(Ay-yOffset);
  Az = zScale*(Az-zOffset);

  pitchRaw = atan2(Ay, sqrt(Az*Az+Ax*Ax))*360/(2*3.14);
  rollRaw = atan2(Ax, sqrt(Az*Az+Ay*Ay))*360/(2*3.14);
  
  //**********Define gyro values**********:
  Gx = g.gyro.x - GyroXOffset;
  Gy = -(g.gyro.y - GyroYOffset);
  Gz = g.gyro.z - GyroZOffset;
  rollG = rollG + (delta_time)/1000.*(Gy)*360./(2.*3.14);
  pitchG = pitchG + (delta_time)/1000.*(Gx)*360./(2.*3.14);
  yawG = yawG + (delta_time)/1000.*Gz*360./(2.*3.14);

  deltaRollG = (delta_time)/1000.*(Gy)*360./(2.*3.14);
  deltaPitchG = (delta_time)/1000.*(Gx)*360./(2.*3.14);
  
  //Implementation of the complimentary filter, takes better of both worlds
  rollComp = Alpha*rollRaw + Beta*(rollComp + deltaRollG);  //Only the value related to the gyros gradient is used
  pitchComp = Alpha*pitchRaw + Beta*(pitchComp + deltaPitchG);
  // Serial.print("Gx:");
  // Serial.print(Gx);
  // Serial.print(",");
  // Serial.print("Gy:");
  // Serial.print(Gy);
  // Serial.print(",");
  // Serial.print("Gz:");
  // Serial.print(Gz);
  // Serial.print(",");
  // Serial.print("UL:");
  // Serial.print(5);
  // Serial.print(",");
  // Serial.print("LM:");
  // Serial.println(-5);
  /*
  Serial.print("RollComp:");
  Serial.print(rollComp);
  Serial.print(",");
  Serial.print("PitchComp:");
  Serial.print(pitchComp);
  Serial.print(",");
  Serial.print("YawG:");  //Yaw cant use the complimentry filter since we dont have its corresponding accelerometer values
  Serial.print(yawG);
  // Serial.print(",");
  // Serial.print("Yaw:");
  // Serial.print(yaw);
  Serial.print(",");
  Serial.print("UL:");
  Serial.print(90);
  Serial.print(",");
  Serial.print("LM:");
  Serial.println(-90);
  */
  last_time = now;
}










