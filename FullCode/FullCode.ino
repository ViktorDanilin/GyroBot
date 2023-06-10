#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define MOTOR_MAX 200
#define MOTOR_MIN 100
// motor PIN's
#define SPEED_1      5 
#define DIR_1        4
 
#define SPEED_2      6
#define DIR_2        7

// consts
#define KDV 30
#define ERROR_MIN KDV*ANGLE_BALANCE_MIN
#define ERROR_MAX KDV*ANGLE_BALANCE_MAX
#define ANGLE_ZERO 3.2
#define TIME_PID 2000
#define TIME_START 500
#define ANGLE_CRASH 20
#define ANGLE_BALANCE_MIN 0.5
#define ANGLE_BALANCE_MAX 5


#define KP 1.4
#define KI 0.04        
#define KD 0.03


MPU6050 accgyro; 

float errorPID;
int  Speed;
long int time_regul;
long int time_start;
float gy;
float angle_ay;
float angleY;
float angleIntegral = 0.0;
boolean isCrash; 
float kp = KP;
float ki = KI;
float kd = KD;

void setup() {
  Serial.begin(9600);
  // настраиваем выводы платы 4, 5, 6, 7 на вывод сигналов 
  for (int i = 4; i < 8; i++) {     
    pinMode(i, OUTPUT);
  }  
  accgyro.initialize();
  calibrate_gyro();
  isCrash = true;
  time_start = millis();
  while(millis() - time_start < TIME_START) {
      getGyroData();
    }
}

void loop() {  
  getGyroData();
  if (isCrash){
    stopMotors(); 
  }else {
    PID_regulator();
  }
}

void PID_regulator() {
  if( time_regul < micros() ){
    time_regul = micros() + TIME_PID;
    angleIntegral +=  (angleY - ANGLE_ZERO);

    errorPID = kp*(angleY - ANGLE_ZERO) + kd*gy + ki*angleIntegral;

    if (errorPID > 0)
      Speed = -map(errorPID*KDV, ERROR_MIN, ERROR_MAX, MOTOR_MIN, MOTOR_MAX);
    else 
      Speed = map(-errorPID*KDV, ERROR_MIN, ERROR_MAX, MOTOR_MIN, MOTOR_MAX);
      
  if ((errorPID >= - ANGLE_BALANCE_MIN) && (errorPID <= ANGLE_BALANCE_MIN))
      Speed = 0;
      
    setSpeedMotors(Speed);    
  }
  Serial.println(String(angleY));
}
void stopMotors(){
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, 0);
  analogWrite(SPEED_2, 0);
}

void setSpeedMotors(int _speed){
  if (abs(_speed)<10) {
      stopMotors();
  } else {
      setSpeedMotorRight(_speed);
      setSpeedMotorLeft(_speed);
  } 
}
void setSpeedMotorLeft(int _speed) {
  int speedNorm = constrain(abs(_speed), 0, MOTOR_MAX);
  if (_speed > 0){
    digitalWrite(DIR_2, LOW);
    analogWrite(SPEED_2, speedNorm);
  }  else {
      digitalWrite(DIR_2, HIGH);
      analogWrite(SPEED_2, speedNorm);
  }
}
void setSpeedMotorRight(int _speed) {
  int speedNorm = constrain(abs(_speed), 0, MOTOR_MAX);
    if (_speed > 0){
    digitalWrite(DIR_1, LOW);
    analogWrite(SPEED_1, speedNorm);
  }  else {
      digitalWrite(DIR_1, HIGH);
      analogWrite(SPEED_1, speedNorm);
  }
}
