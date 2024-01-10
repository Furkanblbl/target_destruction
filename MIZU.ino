#define SAMPLE_RATE 25
#define CONSOLE_DELAY 100
#define COMM_DELAY 50
#define REVERSE 1
#define FORWARD 0
#define CONSOLE_ON 1
#define CONSOLE_OFF 0

#include <QuickPID.h>
#include "SparkFunLSM6DS3.h"
#include <MadgwickAHRS.h>
#include "MS5837.h"
#include "QuickPID.h"
#include <TFLI2C.h>
#include "SerialTransfer.h"
#include "Servo.h"
#include <SoftwareSerial.h>
#include "TFMini.h"
TFMini tfmini;

int valueJoyStick_X_1 = 1500, valueJoyStick_Y_1 = 1500, valueJoyStick_X_2 = 1500, valueJoyStick_Y_2 = 1500;
int RiseM1_target = 1500, RiseM2_target = 1500, FrontRight_target = 1500, FrontLeft_target = 1500, BackRight_target = 1500, BackLeft_target = 1500;
int RiseM1 = 1500, RiseM2 = 1500, FrontRight = 1500, FrontLeft = 1500, BackRight = 1500, BackLeft = 1500;

int Dist1, Dist2, Dist3;
int32_t gyro_change = 0 , gyro_change_prev = 0;
unsigned long microsPerReading, microsPrevious;
unsigned long consolePerReading, consoleMilisPrevious, consoleMillisNow;
unsigned long commPerReading, commMilisPrevious, commMillisNow;
unsigned long lidarMillisNow, lidarMillisPerv, lidaricMillisPerv;
unsigned long motorMillisNow, motorMillisPerv;
unsigned long IMUMillisNow;
float heading1;
float heading2;

float SetpointHeading = 0, InputHeading = 0, OutputHeading = 0;
float Kp =2, Ki = 0.0005, Kd = 0;

float roll, pitch, heading, headingarti, headingOffset;
float pressure, temp, depth, altitude;

bool IMUflag = false;

LSM6DS3 lsm6ds3(I2C_MODE, 0x6A);
MS5837 sensor;
Madgwick filter;
Servo ESc1, ESc2, ESc3, ESc4, ESc5, ESc6;

SerialTransfer myTransfer;
QuickPID pid(&InputHeading, &OutputHeading, &SetpointHeading);

SoftwareSerial SerialTFMini3(13,12);

void setup() {
   
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  
  Serial1.begin(115200);
  Serial2.begin(115200);
  SerialTFMini3.begin(115200);
  
  Serial.begin(4800);
  myTransfer.begin(Serial);
    
  lsm6ds3.begin();
  lsm6ds3.calcGyroOffsets(3000);
  filter.begin(SAMPLE_RATE);
  armESC(3200);

  pinMode(25, OUTPUT);

  SetpointHeading = 0;

  pid.SetTunings(Kp, Ki, Kd);
  pid.SetOutputLimits(-500, 500);
  pid.SetMode(pid.Control::automatic);

  microsPerReading = 1000000 / SAMPLE_RATE;
  microsPrevious = micros();

  consolePerReading = CONSOLE_DELAY;
  consoleMilisPrevious = millis();
  commPerReading = COMM_DELAY;
  commMilisPrevious = millis();

  ESc1.attach(18, 1000, 2000); //RiseSol
  ESc2.attach(19, 1000, 2000); //RiseSað
  ESc3.attach(22, 1000, 2000); //FrontRight
  ESc4.attach(20, 1000, 2000); //FrontLeft
  ESc5.attach(23, 1000, 2000); //BackRight*
  ESc6.attach(24, 1000, 2000); //BackLeft*

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); 
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); 
  delay(1000);
  digitalWrite(10, LOW); 

while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("\n\n\n");
    delay(500);
   } 
  }

void loop() {
  
  IMUSensorValue(CONSOLE_OFF, true);
  pres_sensor_values(CONSOLE_OFF);
  LidarSensorValue(CONSOLE_OFF);
  pid.Compute();
  Comm();
  MotorDrive(CONSOLE_OFF, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD);
}

void LidarSensorValue(bool console) {
  lidarMillisNow = millis();
  if (lidarMillisNow - lidarMillisPerv >= 0.5) {
    lunaDist();
    lidarMillisPerv = lidarMillisNow;
  }
  
  if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {
      Serial.print(" Dist1= ");
      Serial.print(Dist1);
      Serial.println(" cm ");      
      Serial.print(" Dist2= ");
      Serial.print(Dist2);
      Serial.println(" cm ");
      Serial.print(" Dist3= ");
      Serial.print(Dist3);
      Serial.println(" cm ");
      
      consoleMillisNow = consoleMilisPrevious + consolePerReading;
    }
  }
}  

void IMUSensorValue(bool console, bool filterWithMag) {

  float gx, gy, gz, ax, ay, az, mx, my, mz;
  unsigned long microsNow;

  gx = lsm6ds3.readFloatGyroX();
  gy = lsm6ds3.readFloatGyroY();
  gz = lsm6ds3.readFloatGyroZ();
  ax = lsm6ds3.readFloatAccelX();
  ay = lsm6ds3.readFloatAccelY();
  az = lsm6ds3.readFloatAccelZ();
     
 microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    (filterWithMag) ? filter.update(gx, gy, gz, ax, ay, az, mx, my, mz) : filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    heading = heading - 180.0;
    if (heading > 180.0) {
      heading = heading - 360.0;
      if (heading == -360.0) {
        heading = 0;
      }
    }

    IMUMillisNow = millis();
    if (IMUMillisNow >= 10000 && IMUflag == false) {
      headingOffset = heading;
      IMUflag = true;
    }
    if (IMUflag) {
      digitalWrite(25, HIGH); 
      heading += gyro_change; 
      
        if (heading > 180.0) {
        heading = heading - 360.0;
        if (heading == -360.0) {
          heading = 0;
        }
      }
        if (heading < -180.0){
        heading = heading + 360;
      }
    
      InputHeading = heading - headingOffset;

      microsPrevious = microsPrevious + microsPerReading;
    }
  }
  
   if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {

      Serial.print("heading: ");
      Serial.print(heading);
      Serial.print(" pitch: ");
      Serial.print(pitch);
      Serial.print(" roll: ");
      Serial.print(roll);
      Serial.print(" gyroX: ");
      Serial.print(gx);
      Serial.print(" gyroY: ");
      Serial.print(gy);
      Serial.print(" gyroZ: ");
      Serial.print(gz);
      Serial.print(" magX: ");
      Serial.print(mx);
      Serial.print(" magY: ");
      Serial.print(my);
      Serial.print(" magZ: ");
      Serial.println(mz);

      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
    }
  }
}

void updateMotors() {
#define MAX_PWM_CHANGE 2
#define MAX_PWM_CHANGE_RISE 1

  int pwm_difference = RiseM1_target - RiseM1;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE_RISE , MAX_PWM_CHANGE_RISE);
  RiseM1 += pwm_difference;

  pwm_difference = RiseM2_target - RiseM2;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE_RISE, MAX_PWM_CHANGE_RISE);
  RiseM2 += pwm_difference;

  pwm_difference = FrontRight_target - FrontRight;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  FrontRight += pwm_difference;

  pwm_difference = FrontLeft_target - FrontLeft;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  FrontLeft += pwm_difference;

  pwm_difference = BackRight_target - BackRight;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  BackRight += pwm_difference;

  pwm_difference = BackLeft_target - BackLeft;
  pwm_difference = constrain(pwm_difference, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  BackLeft += pwm_difference;

  ESc1.writeMicroseconds(RiseM1);
  ESc2.writeMicroseconds(RiseM2);
  ESc3.writeMicroseconds(FrontRight);
  ESc4.writeMicroseconds(FrontLeft);
  ESc5.writeMicroseconds(BackRight);
  ESc6.writeMicroseconds(BackLeft);
}
void MotorDrive(bool console, bool RiseM1Direct, bool RiseM2Direct, bool FrontRightDirect, bool FrontLeftDirect, bool BackRightDirect, bool BackLeftDirect) {
  int RiseM1_temp, RiseM2_temp, FrontRight_temp, FrontLeft_temp, BackRight_temp, BackLeft_temp;

  RiseM1_temp = valueJoyStick_X_1;  // sol orta
  RiseM2_temp = valueJoyStick_X_1;  // sag orta
  FrontRight_temp = 1500 + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500) + (OutputHeading);
  FrontLeft_temp = 1500 + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500) - (OutputHeading);
  BackRight_temp = 1500 + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500) + (OutputHeading);
  BackLeft_temp = 1500 - (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500) + (OutputHeading);

  if (RiseM1Direct) RiseM1_temp -= 3000;
  if (RiseM2Direct) RiseM2_temp -= 3000;
  if (FrontRightDirect) FrontRight_temp -= 3000;
  if (FrontLeftDirect) FrontLeft_temp -= 3000;
  if (BackRightDirect) BackRight_temp -= 3000;
  if (BackLeftDirect) BackLeft_temp -= 3000;

  RiseM1_temp = constrain(RiseM1_temp, 1000, 2000);
  RiseM2_temp = constrain(RiseM2_temp, 1000, 2000);
  FrontRight_temp = constrain(FrontRight_temp, 1000, 2000);
  FrontLeft_temp = constrain(FrontLeft_temp, 1000, 2000);
  BackRight_temp = constrain(BackRight_temp, 1000, 2000);
  BackLeft_temp = constrain(BackLeft_temp, 1000, 2000);

  RiseM1_target = RiseM1_temp;
  RiseM2_target = RiseM2_temp;
  FrontRight_target = FrontRight_temp;
  FrontLeft_target = FrontLeft_temp;
  BackRight_target = BackRight_temp;
  BackLeft_target = BackLeft_temp;
  motorMillisNow = millis();
  if (motorMillisNow - motorMillisPerv >= 3) {
    updateMotors();
    motorMillisPerv = motorMillisNow;
  }

  if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {

      Serial.print("MotorValue: ");
      Serial.print(RiseM1_temp);
      Serial.print(" ");
      Serial.print(RiseM2_temp);
      Serial.print(" ");


      Serial.print(FrontRight_temp);
      Serial.print(" ");
      Serial.print(FrontLeft_temp);
      Serial.print(" ");
      Serial.print(BackRight_temp);
      Serial.print(" ");
      Serial.println(BackLeft_temp);

      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;

    }
  }
}

void lunaDist() {
  int uart[9];
  int uart2[9];
  int uart3[9];
  int check1,check2,check3;

  if (Serial1.available()) {       //check if serial port has data input
    if (Serial1.read() == 0x59) {  //assess data package frame header 0x59
      uart[0] = 0x59;
      if (Serial1.read() == 0x59) {  //assess data package frame header 0x59
        uart[1] = 0x59;
        for (int i = 2; i < 9; i++) {  //save data in array
          uart[i] = Serial1.read();
        }
        check1 = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check1 & 0xff)) {    //verify the received data as per protocol
          Dist1 = uart[2] + uart[3] * 256;  //calculate distance value
            
        }
      }
    }
  }
  
  if (Serial2.available()) {       //check if serial port has data input
    if (Serial2.read() == 0x59) {  //assess data package frame header 0x59
      uart2[0] = 0x59;
      if (Serial2.read() == 0x59) {  //assess data package frame header 0x59
        uart2[1] = 0x59;
        for (int i = 2; i < 9; i++) {  //save data in array
          uart2[i] = Serial2.read();
        }
        check2 = uart2[0] + uart2[1] + uart2[2] + uart2[3] + uart2[4] + uart2[5] + uart2[6] + uart2[7];
        if (uart2[8] == (check2 & 0xff)) {    //verify the received data as per protocol
          Dist2 = uart2[2] + uart2[3] * 256;  //calculate distance value
            
        }
      }
    }
  }
  
  if (SerialTFMini3.available()) {       //check if serial port has data input
    if (SerialTFMini3.read() == 0x59) {  //assess data package frame header 0x59
      uart3[0] = 0x59;
      if (SerialTFMini3.read() == 0x59) {  //assess data package frame header 0x59
        uart3[1] = 0x59;
        for (int i = 2; i < 9; i++) {  //save data in array
          uart3[i] = SerialTFMini3.read();
        }
        check3 = uart3[0] + uart3[1] + uart3[2] + uart3[3] + uart3[4] + uart3[5] + uart3[6] + uart3[7];
        if (uart3[8] == (check3 & 0xff)) {    //verify the received data as per protocol
          Dist3 = uart3[2] + uart3[3] * 256;  //calculate distance value
            
        }
      }
    }
  }
}

void Comm() {
  struct STRUCT {
    int32_t RollS = 0;
    int32_t PitchS = 0;
    int32_t HeadingS = 0;
    int32_t frontLidarS = 0;
    int32_t leftLidarS = 0;
    int32_t rightLidarS = 0;
    int32_t altitudeS = 0;
    int32_t data1S = 0;
    int32_t data2S = 0;
  } rovDataTx;

  struct STRUCT1 {
    int32_t leftThumbX;
    int32_t leftThumbY;
    int32_t rightThumbX;
    int32_t rightThumbY;
    int32_t degree;
  } rovDataRx;

  rovDataTx.RollS = int(headingOffset);//roll
  rovDataTx.PitchS = int(heading); //pitch
  rovDataTx.HeadingS = int(InputHeading);
  rovDataTx.frontLidarS = int(Dist1); 
  rovDataTx.leftLidarS = int(Dist2); 
  rovDataTx.rightLidarS = int(Dist3);
  rovDataTx.altitudeS = int(depth);
  rovDataTx.data1S = valueJoyStick_X_2;

  if (myTransfer.available()) {
    commMillisNow = millis();
    if (commMillisNow - commMilisPrevious >= commPerReading) {
      uint16_t recSize = 0;
      uint16_t sendSize = 0;

      recSize = myTransfer.rxObj(rovDataRx, recSize);

      sendSize = myTransfer.txObj(rovDataTx, sendSize);

      myTransfer.sendData(sendSize);
      commMilisPrevious = commMilisPrevious + commPerReading;
    }
    gyro_change = rovDataRx.degree;
    valueJoyStick_X_2 = map(rovDataRx.leftThumbX, -320, 320, 1000, 2000);
    valueJoyStick_X_1 = map(rovDataRx.leftThumbY, -320, 320, 1000, 2000);
    valueJoyStick_Y_2 = map(rovDataRx.rightThumbX, -320, 320, 1000, 2000);
    valueJoyStick_Y_1 = map(rovDataRx.rightThumbY, -320, 320, 1000, 2000);
  }
}

void pres_sensor_values(bool console) {
  lidarMillisNow = millis();
  if (lidarMillisNow - lidaricMillisPerv >= 500) {
    sensor.read();
    depth = sensor.depth();
    lidaricMillisPerv = lidarMillisNow;
  }

  if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {
    
      Serial.print("Depth: ");
      Serial.print(sensor.depth());
      Serial.println(" m");
 
      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
    }
  }
}

void armESC(int delayESC) {

  ESc1.writeMicroseconds(1500);
  ESc2.writeMicroseconds(1500);
  ESc3.writeMicroseconds(1500);
  ESc4.writeMicroseconds(1500);
  ESc5.writeMicroseconds(1500);
  ESc6.writeMicroseconds(1500);

  delay(delayESC);
}