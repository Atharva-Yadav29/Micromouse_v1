#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <SparkFun_TB6612.h>

// --- Motor driver setup ---
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 27
#define BIN2 26
#define PWMB 14
Motor motorLeft(AIN1, AIN2, PWMA,-1, 99);
Motor motorRight(BIN1, BIN2, PWMB,-1, 99);

// --- Encoders (Quadrature) ---
#define ENCL_A 35
#define ENCL_B 34
#define ENCR_A 39
#define ENCR_B 36

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// --- I2C for Sensors ---
#define SDA_LEFT   22
#define SCL_LEFT   23 
#define SDA_RIGHT  4
#define SCL_RIGHT  16
#define IR_Front 5
TwoWire I2C_Left(0);
TwoWire I2C_Right(1);


Adafruit_APDS9960 sensorLeft;
Adafruit_APDS9960 sensorRight;


// --- INT Pins for Sensors ---
#define INT_LEFT 21
#define INT_RIGHT 17

volatile bool leftTriggered = false;
volatile bool rightTriggered = false;

bool leftwall=false;
bool rightwall=false;
// --- PID variables ---
float encoderKp = 0.5, encoderKi = 0.0, encoderKd = 0.02;
float wallKp = 6.0, wallKi = 0.0, wallKd = 0.1;

float encoderError = 0, encoderPrev = 0, encoderInt = 0;
float wallError = 0, wallPrev = 0, wallInt = 0;

int baseSpeed = 230;
float encoderPIDValue = 0;
float wallPIDValue = 0;

// --- Interrupt Service Routines ---
void IRAM_ATTR handleLeftA() {
  if (digitalRead(ENCL_A) == digitalRead(ENCL_B)) encoderCountLeft++;
  else encoderCountLeft--;
}

void IRAM_ATTR handleRightA() {
  if (digitalRead(ENCR_A) == digitalRead(ENCR_B)) encoderCountRight++;
  else encoderCountRight--;
}

void IRAM_ATTR handleLeftSensorINT() {
  leftTriggered = true;
}
void IRAM_ATTR handleRightSensorINT() {
  rightTriggered = true;
}


void setMotor(int rspeed, int lspeed) {
  motorLeft.drive(lspeed);
  motorRight.drive(rspeed);
}
void turnRight(){
  setMotor(-150,150);
  delay(100);
}
void turnLeft(){
  setMotor(150,-150);
  delay(100);
}
void stopMotors(){
  setMotor(0,0);
  delay(50);
}
void setup() {
  Serial.begin(115200);

  // --- Encoder pins ---
  pinMode(ENCL_A, INPUT);
  pinMode(ENCL_B, INPUT);
  pinMode(ENCR_A, INPUT);
  pinMode(ENCR_B, INPUT);
  pinMode(IR_Front,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), handleRightA, CHANGE);

  // --- I2C Buses ---
  I2C_Left.begin(SDA_LEFT, SCL_LEFT);
  I2C_Right.begin(SDA_RIGHT, SCL_RIGHT);

  // --- Sensor Init (8X Gain) ---
  // --- Sensor Init ---
sensorLeft.begin(10, APDS9960_AGAIN_1X, 0x39, &I2C_Left);
sensorRight.begin(10, APDS9960_AGAIN_1X, 0x39, &I2C_Right);

// Set Proximity Gain to 8X
sensorLeft.setProxGain(APDS9960_PGAIN_8X);
sensorRight.setProxGain(APDS9960_PGAIN_8X);

  sensorLeft.enableProximity(true);
  sensorRight.enableProximity(true);

  // --- INT Pins ---
  pinMode(INT_LEFT, INPUT_PULLUP);
  pinMode(INT_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INT_LEFT), handleLeftSensorINT, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_RIGHT), handleRightSensorINT, FALLING);

  // --- Increased detection range ---
  sensorLeft.setProximityInterruptThreshold(0, 400);   // Default was 200
  sensorRight.setProximityInterruptThreshold(0, 400);

  sensorLeft.enableProximityInterrupt();
  sensorRight.enableProximityInterrupt();
}

void loop() {
  uint8_t proxL = sensorLeft.readProximity();
    uint8_t proxR = sensorRight.readProximity();
  Serial.print("ProxL: ");
Serial.println(proxL);

Serial.print("ProxR: ");
Serial.println(proxR);
//pid();

}
// --- PID Functions ---
void encoderPID() {
  long leftTicks = encoderCountLeft;
  long rightTicks = encoderCountRight;
  encoderCountLeft = 0;
  encoderCountRight = 0;

  encoderError = leftTicks - rightTicks;
  encoderInt += encoderError;
  float encoderDeriv = encoderError - encoderPrev;
  encoderPrev = encoderError;
  encoderPIDValue = encoderKp * encoderError + encoderKi * encoderInt + encoderKd * encoderDeriv;

  
}

void wallPID(uint8_t proxL, uint8_t proxR) {
  wallError = (int)proxL - (int)proxR;
  wallInt += wallError;
  float wallDeriv = wallError - wallPrev;
  wallPrev = wallError;
  wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;

}
void pid(){
    
    // Read triggered sensor values
    uint8_t proxL = sensorLeft.readProximity();
    uint8_t proxR = sensorRight.readProximity();

    wallPID(proxL, proxR);
    encoderPID();

    float totalCorrection = encoderPIDValue + wallPIDValue;
    int leftSpeed = constrain(baseSpeed + totalCorrection, -255, 255);
    int rightSpeed = constrain(baseSpeed - totalCorrection, -255, 255);
    setMotor(rightSpeed, leftSpeed);
    

  
    
}