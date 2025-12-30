#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SparkFun_TB6612.h>

// Motor control pins
#define AIN1 25
#define AIN2 4
#define PWMA 26
#define BIN1 17
#define BIN2 5
#define PWMB 18
#define STBY 16

// Define motor objects: (IN1, IN2, PWM, offset, standby)
Motor motor_l(AIN1, AIN2, PWMA, -1, 99);
Motor motor_r(BIN1, BIN2, PWMB, -1, 99);

#define ENC_L_A 19
#define ENC_L_B 23
#define ENC_R_A 15
#define ENC_R_B 2

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

int targetSpeed = 200;
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float errorSum = 0, lastError = 0;

float wallKp = 0.0, wallKi = 0.0, wallKd = 0.0;
float wallErrorSum = 0, wallLastError = 0;

unsigned long lastPIDTime = 0;

Adafruit_VL53L0X sensorL, sensorFL, sensorF, sensorFR, sensorR;
#define XSHUT_L 13
#define XSHUT_FL 12
#define XSHUT_F 27
#define XSHUT_FR 33
#define XSHUT_R 32

#define ADDR_L 0x30
#define ADDR_FL 0x31
#define ADDR_F 0x32
#define ADDR_FR 0x33
#define ADDR_R 0x34

int distL, distFL, distF, distFR, distR;

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_L_B)) encoderCountLeft++;
  else encoderCountLeft--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_R_B)) encoderCountRight++;
  else encoderCountRight--;
}

void setupSensors() {
  pinMode(XSHUT_L, OUTPUT); 
  pinMode(XSHUT_FL, OUTPUT);
  pinMode(XSHUT_F, OUTPUT); 
  pinMode(XSHUT_FR, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);

  digitalWrite(XSHUT_L, LOW); 
  digitalWrite(XSHUT_FL, LOW);
  digitalWrite(XSHUT_F, LOW); 
  digitalWrite(XSHUT_FR, LOW); 
  digitalWrite(XSHUT_R, LOW);

  delay(10);

  Wire.begin();

  digitalWrite(XSHUT_L, HIGH); 
  delay(10); 
  sensorL.begin(ADDR_L);

  digitalWrite(XSHUT_FL, HIGH); 
  delay(10); 
  sensorFL.begin(ADDR_FL);

  digitalWrite(XSHUT_F, HIGH); 
  delay(10); 
  sensorF.begin(ADDR_F);

  digitalWrite(XSHUT_FR, HIGH); 
  delay(10); 
  sensorFR.begin(ADDR_FR);

  digitalWrite(XSHUT_R, HIGH);
  delay(10); 
  sensorR.begin(ADDR_R);
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); 
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); 
  digitalWrite(STBY, HIGH);

  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);

  setupSensors();
  lastPIDTime = millis();
}

void loop() {
  static long lastLeft = 0, lastRight = 0;

  unsigned long now = millis();
  if (now - lastPIDTime >= 100) {
    lastPIDTime = now;

    readToFSensors();

if (distR > 250 ) {
  stopMotors();
  turnRight();
  stopMotors();
  return;
}
else if (distF <= 250 && distL > 250) {
  stopMotors();
  turnLeft();
  stopMotors();
  return;
}
else if (distF <= 250) {
  stopMotors();
  uTurn();
  stopMotors();
  return;
}

    float speedCorr = computeEncoderPID(lastLeft, lastRight);
    float wallCorr = computeWallPID();

    int leftPWM = constrain(targetSpeed - speedCorr - wallCorr, 0, 255);
    int rightPWM = constrain(targetSpeed + speedCorr + wallCorr, 0, 255);
    setMotor(rightPWM, leftPWM);
  }
}


void readToFSensors() {
  VL53L0X_RangingMeasurementData_t data;

  sensorL.rangingTest(&data, false); distL = min((int)data.RangeMilliMeter, 800);
  sensorFL.rangingTest(&data, false); distFL = min((int)data.RangeMilliMeter, 800);
  sensorF.rangingTest(&data, false); distF = min((int)data.RangeMilliMeter, 800);
  sensorFR.rangingTest(&data, false); distFR = min((int)data.RangeMilliMeter, 800);
  sensorR.rangingTest(&data, false); distR = min((int)data.RangeMilliMeter, 800);
}

float computeEncoderPID(long &lastLeft, long &lastRight) {
  long countLeft = encoderCountLeft;
  long countRight = encoderCountRight;

  long dLeft = countLeft - lastLeft;
  long dRight = countRight - lastRight;

  lastLeft = countLeft;
  lastRight = countRight;

  long error = dLeft - dRight;
  errorSum += error;
  float dError = error - lastError;
  lastError = error;

  return Kp * error + Ki * errorSum + Kd * dError;
}

float computeWallPID() {
  int targetWallDist = 100;
  float wallError = targetWallDist - distR;
  wallErrorSum += wallError;
  float dWallError = wallError - wallLastError;
  wallLastError = wallError;

  return wallKp * wallError + wallKi * wallErrorSum + wallKd * dWallError;
}

void moveForward(int leftPWM, int rightPWM) {
  setMotor(150,150);//Adjust karna hai
}

void stopMotors() {
  setMotor(0,0);
  delay(200);//(Adjust karna hai)
}

void turnLeft() {
  setMotor(150,-150);
  delay(200);// Adjust karna hai
}
void turnRight() {
  setMotor(-150,150);
  delay(200);// Adjust karna hai
}
void uTurn(){
  setMotor(150,-150);//adjust
}
void setMotor(int set_speed_r, int set_speed_l) {
  motor_r.drive(set_speed_r);
  motor_l.drive(set_speed_l);
}