//*****Wireless Capabilities*****
// Include Libraries
#include <esp_now.h>
#include <WiFi.h>
 
// Define a data structure
typedef struct struct_message {
  float a;
  float b;
  float c;
  float d;
} struct_message;
 
// Create a structured object
struct_message myData;

//*****Actual controller setup*****

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

//Motor Pins:
const int pot_Pin = A0;  //defines pin for potentiaometer

//Pins for motor A:
const int motorSpeedPin_A = 14;  //defines speed pin of motor
const int IN_1 = 27;  //dfines direction pin of motor
const int IN_2 = 26;  //dfines direction pin of motor

//Pins for motor B:
const int motorSpeedPin_B = 32;  //defines speed pin of motor
const int IN_3 = 33;  //dfines direction pin of motor
const int IN_4 = 25;  //dfines direction pin of motor

//variables for PID controller:
float set_angle = 0;  //in degrees
float angle = 0;      //in degrees
float new_error = 0;
float old_error = 0;
float integral = 0;
float Kp = 0.8;
float Ki = 0.4;
float Kd = 0.3;
int baseDemand = 75;

//Rig dimensions (cm): (Not needed for this code)
float arm_height = 24;
float arm_length = 21;

//Callback function executed when data is received from the transmitter esp32:
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  /*Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("Set Angle: ");
  Serial.print(myData.a);
  Serial.print(", ");
  Serial.print("Kp: ");
  Serial.print(myData.b);
  Serial.print(", ");
  Serial.print("Ki: ");
  Serial.print(myData.c);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.println(myData.d);*/
  set_angle = myData.a;
  Kp = myData.b;
  Ki = myData.c;
  Kd = myData.d;

}

//***** esp32 PWM related stuff *****
const int freq = 5000;           // PWM frequency (Hz)
const int pwmChannel_A = 0;      // Channel for motor A
const int pwmChannel_B = 1;      // Channel for motor B
const int resolution = 8;                  //255 resolution

void setup() {
  // put your setup code here, to run once:

  //Start serial monitor:
  Serial.begin(115200);

  //Set esp-now:
  //Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  //Set Motor pins:
  pinMode(pot_Pin, INPUT);
  pinMode(motorSpeedPin_A, OUTPUT);
  pinMode(motorSpeedPin_B, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  // ========== PWM setup with ledc for esp32 ==========

  ledcAttach(motorSpeedPin_A, freq, resolution);  // Attach pin + set freq & resolution
  ledcAttach(motorSpeedPin_B, freq, resolution);

  //Set the mpu:
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1);
  }
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

    //set_angle = (analogRead(pot_Pin) / 1023.) * 40.;  //Get the set height

    Get_IMU_Data(pitchComp);  //Get imu orientation, Note the angle is extracted using referencing

    Serial.print("Set_angle:");
    Serial.print(set_angle);
    Serial.print(", ");
    Serial.print("Angle:");
    Serial.print(pitchComp);
    Serial.print(", ");


    new_error = Get_Error(set_angle, pitchComp);
    Motor_Controller(Kp, Ki, Kd, new_error, old_error, integral, interval, baseDemand, motorSpeedPin_A, motorSpeedPin_B, IN_1, IN_2, IN_3, IN_4);

    Serial.print("Time:");
    Serial.print(current_time/1000.0f);
    Serial.print(", ");    
    Serial.print("PitchComp:");
    Serial.print(pitchComp);
    // Serial.print(",");
    // Serial.print("Yaw:");
    // Serial.print(yaw);
    Serial.print(", ");
    Serial.print("UL:");
    Serial.print(90);
    Serial.print(", ");
    Serial.print("LM:");
    Serial.print(-90);
    Serial.print(", ");
    
    old_time = current_time;
  }

}

//User defined functions are shown bellow: (These include only the ones used in the main loop)

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

void Get_IMU_Data(float &pitchComp) {
  
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

float Get_Error(float set_angle, float angle) {
  //code for finding error:
  float error = set_angle - angle;
  
  return error;
}

void Motor_Controller(float Kp, float Ki, float Kd, float new_error, float &old_error, float &integral, float interval, float base_demand, int motorSpeedPin_A, int motorSpeedPin_B, int IN_1, int IN_2, int IN_3, int IN_4) {
  //PID code:
  float integral_increment = new_error * ((interval)/(1000.));  //change in integral term
  float derivative = (new_error - old_error) / ((interval)/(1000.)); // Rate of change in error
  
  float test_integral = integral + integral_increment;
  float test_motorDemand_A = base_demand + (Kp*new_error + Ki*test_integral + Kd*derivative);
  float test_motorDemand_B = base_demand - (Kp*new_error + Ki*test_integral + Kd*derivative);

  //Anti windup system: (shuts down integrator is the case of saturation getting worse)

  //Anti windup for motor A:
  if ((fabs(test_motorDemand_A) > 200) || (fabs(test_motorDemand_B) > 200)) {  //check if motorDemand for motor has reached saturation e.g. outside -100 to 100
    if ((new_error * test_motorDemand_A > 0) || (new_error * test_motorDemand_B < 0)) {  //The sign is flipped for motor B because the PID is minused from it.
        //if saturation does hit and integrator is making it worse dont update integral by doing nothing

    }
    else {
      //if saturation does hit but integrator isn't making it worse by going even more outside
      integral = integral + integral_increment;

    }
  }
  else {
    //if saturation isn't hit than act like its normal integration
    integral = integral + integral_increment;

  }

  //find the final motor demand based on error (correct motor demand after clamp check);
  float motorDemand_A = base_demand + (Kp*new_error + Ki*integral + Kd*derivative);  //Motor A demand
  float motorDemand_B = base_demand - (Kp*new_error + Ki*integral + Kd*derivative);  //Motor B demand (As one increases the other must decrease)

  motorDemand_A = constrain(motorDemand_A, -200, 200);  //clamp the motorDemand
  motorDemand_B = constrain(motorDemand_B, -200, 200);  //clamp the motorDemand
  
  //Motor A assigning speed:
  if (motorDemand_A < 0 ) {
    //switch direction to backwards:
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
  }

  else{
    //switch direction to fowards:
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);    
  }
  ledcWrite(motorSpeedPin_A, fabs(motorDemand_A));  //set the speed to motor A

  //Motor B assigning speed:
  if (motorDemand_B < 0 ) {
    //switch direction to backwards:
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
  }

  else{
    //switch direction to fowards:
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);    
  }
  ledcWrite(motorSpeedPin_B, fabs(motorDemand_B));  //set the speed to motor B

  Serial.print("Error:");
  Serial.print(new_error);
  Serial.print(", ");
  Serial.print("Motor_Demand_A:");
  Serial.print(motorDemand_A);
  Serial.print(", ");
  Serial.print("Motor_Demand_B:");
  Serial.print(motorDemand_B);
  Serial.print(", ");
  Serial.print("Proportional:");
  Serial.print(Kp*new_error);
  Serial.print(", ");  
  Serial.print("Integral:");
  Serial.print(Ki*integral);
  Serial.print(", ");
  Serial.print("Derivative:");
  Serial.println(Kd*derivative);

  old_error = new_error;
}

// Code made by Sheil...







