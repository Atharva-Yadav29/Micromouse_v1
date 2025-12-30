/*
 * Micromouse: Floodfill Robot (Real Hardware Code)
 * Tailored for ESP32 with TB6612FNG, dual APDS9960 sensors, and N20 encoders.
 *
 * IMPORTANT: You MUST calibrate the values in the "Robot Physical Constants"
 * and "Sensor Calibration" sections for your robot to work correctly.
 */

#include <Arduino.h>
#include <Wire.h>              // For I2C communication
#include "Adafruit_VL53L0X.h"  // For the side wall sensors
#include <SparkFun_TB6612.h>   // For the motor driver
#include <queue>
// =================================================================
// SIMULATION FLAG: Set to 'false' to compile for the actual robot
#define SIMULATION_MODE false
// =================================================================
// Global variables to hold sensor readings
unsigned int leftDistance = 0;   // Distance in mm from left VL53L0X
unsigned int rightDistance = 0;  // Distance in mm from right VL53L0X


unsigned int proxL;  // Changed to unsigned int to hold values up to 500
unsigned int proxR;  // Changed to unsigned int to hold values up to 500
uint8_t frontVal;    // This can remain as is
// --- Pin Definitions (from your image) ---

// Motor Driver Pins (TB6612FNG)
#define AIN1 33
#define AIN2 25
#define PWMA 32
#define BIN1 26  // Note: Image shows 26->BIN1, 27->BIN2
#define BIN2 27
#define PWMB 14
#define STBY_PIN 99  // IMPORTANT: Connect the STBY pin of the TB6612FNG to this GPIO

#define LEDR 5
#define LEDL 19

// Encoder Pins (Using one pin per encoder as requested)
#define ENCL_A 35  // Left Encoder pin
#define ENCR_A 39  // Right Encoder pin

// Wall Sensor Pins
//#define IR_FRONT_PIN 18 // Front IR Sensor Output
const int trigPin = 0;
const int echoPin = 18;
long duration;
float distanceCm;
#define SENSOR_TIMEOUT 50000
// I2C Pins for APDS9960 Sensors
#define SDA_LEFT 22
#define SCL_LEFT 23
#define SDA_RIGHT 4
#define SCL_RIGHT 16

// --- Hardware Objects ---
Motor motorLeft = Motor(AIN1, AIN2, PWMA, 1, STBY_PIN);
Motor motorRight = Motor(BIN1, BIN2, PWMB, 1, STBY_PIN);

TwoWire I2C_LEFT = TwoWire(0);   // hardware controller 0
TwoWire I2C_RIGHT = TwoWire(1);  // hardware controller 1
Adafruit_VL53L0X loxLeft;
Adafruit_VL53L0X loxRight;

#define WALL_THRESHOLD 20

// --- Robot Physical Constants ---
// TODO: CRITICAL! Measure and calibrate these values for YOUR robot!
const float WHEEL_DIAMETER_CM = 3.5;     // Diameter of your wheels in cm
const float WHEEL_SEPARATION_CM = 11.0;  // Distance between the centers of the two wheels
const int ENCODER_CPR = 345;             // Counts Per Revolution for your encoders (single channel)
const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / ENCODER_CPR;
#define CELL_DISTANCE_CM 23.0  // Standard micromouse cell size

// --- Movement & Control Parameters ---
// TODO: Tune these values for best performance!
const int BASE_SPEED = 150;           // Base motor PWM value (0-255)
const int TURN_SPEED = 100;           // Speed for turning
const float MOTOR_CORRECTION = 0.93;  // Correction factor if one motor is faster (e.g., right_speed * 0.98)
                                      // Proportional gain for driving straight

float encoderKp = 0.3, encoderKi = 0.0, encoderKd = 0.02;
float encoderError = 0, encoderPrev = 0, encoderInt = 0;

float wallKp = 1.40, wallKi = 0.0, wallKd = 3.0;
float wallError = 0, wallPrev = 0, wallInt = 0;
float encoderPIDValue = 0;
float wallPIDValue = 0;
// The speed the robot will travel at for most of the cell
const int MAX_SPEED = 150;

// The minimum speed the robot will slow down to just before stopping
const int MIN_SPEED = 110;

const float SLOW_RATE = 2.5;

// How far from the target (in encoder ticks) the robot should start slowing down.
// A larger value means a more gradual, smoother slowdown.
// A smaller value means a more abrupt slowdown.
const long DECELERATION_TICKS = 500;
// --- Sensor Calibration ---
// TODO: CRITICAL! Calibrate these thresholds by taking readings!
const int LEFT_WALL_THRESHOLD = 60;   // Proximity value from APDS9960 (0-255)
const int RIGHT_WALL_THRESHOLD = 60;  // Proximity value from APDS9W960 (0-255)
const int FRONT_WALL_THRESHOLD = 10;  // Value from front IR sensor (e.g., 0-4095 for analog)
int RIGHT_WALL_TARGET = 80;
int LEFT_WALL_TARGET = 85;  // --- Global Variables ---
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// --- Maze / Floodfill Data Structures ---
const int GRID_SIZE = 5;
int distMap[GRID_SIZE][GRID_SIZE];
uint8_t wallsMap[GRID_SIZE][GRID_SIZE];
bool visited[GRID_SIZE][GRID_SIZE];

#define WALL_N 1
#define WALL_E 2
#define WALL_S 4
#define WALL_W 8

enum Heading { NORTH = 0,
               EAST = 1,
               SOUTH = 2,
               WEST = 3 };
int robotX = 2, robotY = 0;
Heading robotHeading = NORTH;

bool goalReached = false;
// For a 5x5 grid (indices 0-4), the center is (2,2)
int goalX = 4, goalY = 4;
// --- Function Prototypes ---
void attachHardware();
void computeFloodfill(int gx, int gy);
bool nextCellTowardsGoal(int &nx, int &ny);
void senseWallsAtCurrentCell();
void executeMoveTo(int nx, int ny);
void moveForwardOneCell();
void turn(int degrees);
void pathReturnToStart();
bool inBounds(int x, int y);
void stopMotors();
int readLeftProx();
int readRightProx();
float readFrontSensor();
void IRAM_ATTR isrLeftEncoder();
void IRAM_ATTR isrRightEncoder();

// --- State Machine ---
enum ProgramState {
  EXPLORING,
  AWAITING_FAST_RUN,
  FAST_RUN,
  FINISHED
};
ProgramState currentState = EXPLORING;

#define BUTTON_PIN 2  // TODO: Connect a button to this GPIO pin and ground
// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Micromouse Hardware Mode Initializing...");

  attachHardware();

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      wallsMap[y][x] = 0;
      distMap[y][x] = 10000;
      visited[y][x] = false;
    }
  }
  for (int y = 0; y < GRID_SIZE; y++) {
    wallsMap[y][0] |= WALL_W;
    wallsMap[y][GRID_SIZE - 1] |= WALL_E;
  }
  for (int x = 0; x < GRID_SIZE; x++) {
    wallsMap[0][x] |= WALL_S;
    wallsMap[GRID_SIZE - 1][x] |= WALL_N;
  }

  robotX = 0;
  robotY = 0;
  robotHeading = NORTH;
  visited[robotY][robotX] = true;

  computeFloodfill(goalX, goalY);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configure the button pin

  Serial.println("State: EXPLORING. Ready to map the maze...");
  delay(3000);
}

// ------------------- Main loop -------------------
void loop() {

  readAllSensors();
  Serial.print("Sensor values: L=");
  Serial.print(proxL);
  Serial.print(" R=");
  Serial.print(proxR);
  Serial.println("");
  // wallPID(proxL, proxR);
  // float error = WALL_TARGET - (float)proxR;
  //       float    wallCorrection = wallKp * error;  
  //       // float wallCorrection = wallPIDValue;
  //       // encoderPID();
  //           Serial.println("no walls detected");
  //           float totalCorrection =  wallCorrection;  //encoderPIDValue +
  //         int leftSpeed = constrain(BASE_SPEED - totalCorrection, -255, 255);
  //         int rightSpeed = constrain(BASE_SPEED+ totalCorrection, -255, 255);

  //         setMotorSpeeds(rightSpeed, leftSpeed);
  //moveForwardOneCell();
  //    delay(2000);

  //  senseWallsAtCurrentCell();

  // switch (currentState) {
  //   case EXPLORING:
  //     runExploration();
  //     break;

  //   case AWAITING_FAST_RUN:
  //     // Robot is at the start, waiting for the button press.
  //     // You can blink an LED here to show it's ready.
  //       stopMotors();
  //       delay(5000);
  //     // if (digitalRead(BUTTON_PIN) == LOW) {  // Button is pressed
  //     //   delay(50);                           // Debounce
  //     //   Serial.println("\nButton pressed! Starting FAST RUN!");
  //       currentState = FAST_RUN;
  //     //}
  //     break;

  //   case FAST_RUN:
  //     performFastRun();         // Execute the speed run
  //     currentState = FINISHED;  // After the run, go to the finished state
  //     break;

  //   case FINISHED:
  //     // The robot has completed all tasks. Stop and do nothing.
  //     stopMotors();
  //     delay(1000);
  //     break;
  // }
}

// =================================================================
// ================= HARDWARE-SPECIFIC FUNCTIONS ===================
// =================================================================

void attachHardware() {
  // --- Motor Driver Pin ---
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);  // Enable motor driver

  // --- Sensor Pins ---
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDL, OUTPUT);

  I2C_LEFT.begin(SDA_LEFT, SCL_LEFT, 500000);  // 400 kHz fast-mode
  I2C_RIGHT.begin(SDA_RIGHT, SCL_RIGHT, 500000);
  if (!loxLeft.begin(0x29, false, &I2C_LEFT)) {
    Serial.println(F("Left VL53L0X not found"));
    while (1)
      ;
  }
  if (!loxRight.begin(0x29, false, &I2C_RIGHT)) {
    Serial.println(F("Right VL53L0X not found"));
    while (1)
      ;
  }

  Serial.println("Sensors initialized");

  // --- Encoder Pins & Interrupts ---
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), isrLeftEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), isrRightEncoder, FALLING);

  Serial.println("Hardware attached.");
}


float readFrontSensor() {
  // To generate a clean pulse, first make sure the trigger pin is low
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin with a timeout to prevent watchdog crashes
  duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);

  // If a timeout occurs, pulseIn() returns 0. Handle this case.
  if (duration == 0) {
    // You can return a specific value to indicate "out of range"
    distanceCm = 40;
  } else {
    // Calculate the distance if a valid pulse was received
    distanceCm = duration * 0.0343 / 2;
  }

  return distanceCm;
}

void stopMotors() {
  motorLeft.brake();
  motorRight.brake();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed * MOTOR_CORRECTION);
}

// ------------------- Sensing Logic -------------------
void senseWallsAtCurrentCell() {  // Small delay for sensor readings to stabilize
  uint8_t cellWalls = wallsMap[robotY][robotX];
  readAllSensors();
  int leftVal = proxL;
  int rightVal = proxR;

  // Serial.print("Sensor values: L=");
  // Serial.print(leftVal);
  // Serial.print(" R=");
  // Serial.print(rightVal);
  // Serial.print(" F=");
  // Serial.println(frontVal);

  bool leftWall = leftVal > LEFT_WALL_THRESHOLD;
  bool rightWall = rightVal > RIGHT_WALL_THRESHOLD;
  bool frontWall = frontVal < FRONT_WALL_THRESHOLD;
  if (leftWall) {
    Serial.println("Left Wall");
    delay(100);
  }
  if (frontWall) {
    Serial.println("front Wall");
    delay(100);
  }
  if (rightWall) {
    Serial.println("right Wall");
    delay(100);
  }
  Serial.println("==============================================================");
  if (leftWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_W;
    else if (robotHeading == EAST) cellWalls |= WALL_N;
    else if (robotHeading == SOUTH) cellWalls |= WALL_E;
    else cellWalls |= WALL_S;
  }
  if (rightWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_E;
    else if (robotHeading == EAST) cellWalls |= WALL_S;
    else if (robotHeading == SOUTH) cellWalls |= WALL_W;
    else cellWalls |= WALL_N;
  }
  if (frontWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_N;
    else if (robotHeading == EAST) cellWalls |= WALL_E;
    else if (robotHeading == SOUTH) cellWalls |= WALL_S;
    else cellWalls |= WALL_W;
  }

  wallsMap[robotY][robotX] = cellWalls;

  if (cellWalls & WALL_N)
    if (inBounds(robotX, robotY + 1)) wallsMap[robotY + 1][robotX] |= WALL_S;
  if (cellWalls & WALL_E)
    if (inBounds(robotX + 1, robotY)) wallsMap[robotY][robotX + 1] |= WALL_W;
  if (cellWalls & WALL_S)
    if (inBounds(robotX, robotY - 1)) wallsMap[robotY - 1][robotX] |= WALL_N;
  if (cellWalls & WALL_W)
    if (inBounds(robotX - 1, robotY)) wallsMap[robotY][robotX - 1] |= WALL_E;
}

// The function now takes pointers to 'long' to hold the mapped values
/**
 * Reads all sensors, maps the proximity values to a 0-500 range,
 * and updates the global variables.
 */
void readAllSensors() {
  VL53L0X_RangingMeasurementData_t leftData, rightData;

  // Read both VL53L0X sensors
  loxLeft.rangingTest(&leftData, false);
  loxRight.rangingTest(&rightData, false);

  if (leftData.RangeStatus != 4) {  // 4 = out of range
    leftDistance = leftData.RangeMilliMeter;
  } else {
    leftDistance = 300;  // Set to max value if out of range
  }

  if (rightData.RangeStatus != 4) {  // 4 = out of range
    rightDistance = rightData.RangeMilliMeter;
  } else {
    rightDistance = 300;  // Set to max value if out of range
  }

  // Convert distance to proximity values (invert: closer = higher value)
  // This makes it compatible with the original APDS9960 proximity logic
  proxL = constrain(map(leftDistance, 300, 50, 0, 100), 0, 100);
  proxR = constrain(map(rightDistance, 300, 50, 0, 100), 0, 100);

  frontVal = readFrontSensor();
}
// ------------------- Movement Logic -------------------
void moveForwardOneCell() {
  long targetTicks = CELL_DISTANCE_CM / CM_PER_TICK;
  noInterrupts();
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();

  // Reset PID controllers at the start of the move
  encoderInt = 0;
  encoderPrev = 0;
  wallInt = 0;
  wallPrev = 0;

  while (true) {
    long avgTicks = (encoderCountLeft + encoderCountRight) / 2;

    if (avgTicks >= targetTicks) break;  // Exit when distance is reached

    // --- Speed Control ---
    float progress = avgTicks / (float)targetTicks;
    int dynamicSpeed = BASE_SPEED - pow(progress, SLOW_RATE) * (BASE_SPEED - TURN_SPEED);
    dynamicSpeed = constrain(dynamicSpeed, TURN_SPEED, BASE_SPEED);

    // --- PID Control Loop ---
    readAllSensors();
    if(frontVal<10){
  break;
}

    // *FIX*: Reset wall PID value each loop to prevent using stale data
    wallPIDValue = 0;

    bool hasLeftWall = proxL > LEFT_WALL_THRESHOLD;
    bool hasRightWall = proxR > RIGHT_WALL_THRESHOLD;

    // --- Wall Following Logic ---
    if (hasLeftWall && hasRightWall) {
      // Case 1: Both walls are present. Center the robot between them.
      // Serial.println("Both walls detected");
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, HIGH);
      wallPID(proxL, proxR);  // This function calculates the error between L and R
    } else if (hasRightWall) {
      // Case 2: Only the right wall is present. Follow it at a target distance.
      // Serial.println("Only right wall detected");
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, HIGH);
      wallError = RIGHT_WALL_TARGET - (float)proxR;  // Calculate error from the ideal distance
      wallInt += wallError;
      float wallDeriv = wallError - wallPrev;
      wallPrev = wallError;
      wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;
    } else if (hasLeftWall) {
      // Case 3: Only the left wall is present. Follow it at a target distance.
      // Serial.println("Only left wall detected");
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, LOW);
      wallError = (float)proxL - LEFT_WALL_TARGET;  // Calculate error from the ideal distance
      wallInt += wallError;
      float wallDeriv = wallError - wallPrev;
      wallPrev = wallError;
      wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;
    } else {
      // Case 4: No walls. Turn off LEDs and rely only on encoders.
      // Serial.println("No walls detected");
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, LOW);
      // wallPIDValue is already 0, so no wall correction will be applied.
    }

    // --- Encoder PID (to keep wheels synchronized) ---
    encoderPID();

    // *FIX*: Combine both PID corrections.
    // This is the most critical fix.
    float encoderWeight = 0.4;  // Prioritize encoders for straight driving
    float wallWeight = 0.6;     // Use walls for minor course correction
    float totalCorrection = (encoderWeight * encoderPIDValue) + (wallWeight * wallPIDValue);

    // --- Apply Motor Speeds ---
    int leftSpeed = constrain(dynamicSpeed - totalCorrection, -255, 255);
    int rightSpeed = constrain(dynamicSpeed + totalCorrection, -255, 255);

    setMotorSpeeds(rightSpeed, leftSpeed);
  }

  // Ensure motors fully stop after the move
  stopMotors();
  delay(50);
}
void moveForwardOneCellFast() {
  long targetTicks = CELL_DISTANCE_CM / CM_PER_TICK;
  noInterrupts();
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();

  // Reset PID controllers at the start of the move
  encoderInt = 0;
  encoderPrev = 0;
  wallInt = 0;
  wallPrev = 0;

  while (true) {
    long avgTicks = (encoderCountLeft + encoderCountRight) / 2;

    if (avgTicks >= targetTicks) break;  // Exit when distance is reached

    // --- Speed Control ---
    float progress = avgTicks / (float)targetTicks;
    int dynamicSpeed = BASE_SPEED - pow(progress, SLOW_RATE) * (BASE_SPEED - TURN_SPEED);
    dynamicSpeed = constrain(dynamicSpeed, TURN_SPEED, BASE_SPEED);

    // --- PID Control Loop ---
    readAllSensors();
    if(frontVal<10){
  break;
}

    // *FIX*: Reset wall PID value each loop to prevent using stale data
    wallPIDValue = 0;

    bool hasLeftWall = proxL > LEFT_WALL_THRESHOLD;
    bool hasRightWall = proxR > RIGHT_WALL_THRESHOLD;

    // --- Wall Following Logic ---
    if (hasLeftWall && hasRightWall) {
      // Case 1: Both walls are present. Center the robot between them.
      // Serial.println("Both walls detected");
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, HIGH);
      wallPID(proxL, proxR);  // This function calculates the error between L and R
    } else if (hasRightWall) {
      // Case 2: Only the right wall is present. Follow it at a target distance.
      // Serial.println("Only right wall detected");
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, HIGH);
      wallError = RIGHT_WALL_TARGET - (float)proxR;  // Calculate error from the ideal distance
      wallInt += wallError;
      float wallDeriv = wallError - wallPrev;
      wallPrev = wallError;
      wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;
    } else if (hasLeftWall) {
      // Case 3: Only the left wall is present. Follow it at a target distance.
      // Serial.println("Only left wall detected");
      digitalWrite(LEDL, HIGH);
      digitalWrite(LEDR, LOW);
      wallError = (float)proxL - LEFT_WALL_TARGET;  // Calculate error from the ideal distance
      wallInt += wallError;
      float wallDeriv = wallError - wallPrev;
      wallPrev = wallError;
      wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;
    } else {
      // Case 4: No walls. Turn off LEDs and rely only on encoders.
      // Serial.println("No walls detected");
      digitalWrite(LEDL, LOW);
      digitalWrite(LEDR, LOW);
      // wallPIDValue is already 0, so no wall correction will be applied.
    }

    // --- Encoder PID (to keep wheels synchronized) ---
    encoderPID();

    // *FIX*: Combine both PID corrections.
    // This is the most critical fix.
    float encoderWeight = 0.4;  // Prioritize encoders for straight driving
    float wallWeight = 0.6;     // Use walls for minor course correction
    float totalCorrection = (encoderWeight * encoderPIDValue) + (wallWeight * wallPIDValue);

    // --- Apply Motor Speeds ---
    int leftSpeed = constrain(dynamicSpeed - totalCorrection, -255, 255);
    int rightSpeed = constrain(dynamicSpeed + totalCorrection, -255, 255);

    setMotorSpeeds(rightSpeed, leftSpeed);
  }
}

void wallPID(unsigned int proxL, unsigned int proxR) {
  wallError = -(int)proxR + (int)proxL;
  wallInt += wallError;
  float wallDeriv = wallError - wallPrev;
  wallPrev = wallError;
  wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;
}

// Calculates correction to keep wheels synchronized
void encoderPID() {
  encoderError = encoderCountLeft - encoderCountRight;
  encoderInt += encoderError;
  float encoderDeriv = encoderError - encoderPrev;
  encoderPrev = encoderError;
  encoderPIDValue = encoderKp * encoderError + encoderKi * encoderInt + encoderKd * encoderDeriv;
}

void turn(int degrees) {
  float wheel_circumference = PI * WHEEL_DIAMETER_CM;
  float turn_circumference = PI * WHEEL_SEPARATION_CM;
  float distance_per_wheel = ((float)abs(degrees) / 360.0) * turn_circumference;
  long targetTicks = distance_per_wheel / wheel_circumference * ENCODER_CPR;

  noInterrupts();
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();

  if (degrees > 0) {  // Turn right
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  } else {  // Turn left
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  }

  while ((abs(encoderCountLeft) + abs(encoderCountRight)) / 2 < targetTicks) {
    delay(5);
  }
  stopMotors();

  // Update heading
  int turns = degrees / 90;
  robotHeading = (Heading)((robotHeading + turns + 4) % 4);
}


void executeMoveTo(int nx, int ny) {
  int dx = nx - robotX;
  int dy = ny - robotY;
  Heading target = robotHeading;
  if (dx == 1 && dy == 0) target = EAST;
  else if (dx == -1 && dy == 0) target = WEST;
  else if (dx == 0 && dy == 1) target = NORTH;
  else if (dx == 0 && dy == -1) target = SOUTH;

  int delta = (target - robotHeading + 4) % 4;
  // Use exact 90° increments so heading update is unambiguous
  if (delta == 1) { turn(90); }  // Turn Right
  else if (delta == 2) {
    turn(180);
  }                                    // Turn Around
  else if (delta == 3) { turn(-90); }  // Turn Left
  // if delta==0 do nothing

  moveForwardOneCell();

  robotX = nx;
  robotY = ny;
  visited[robotY][robotX] = true;
}
void executeMoveToFast(int nx, int ny) {
  int dx = nx - robotX;
  int dy = ny - robotY;
  Heading target = robotHeading;
  if (dx == 1 && dy == 0) target = EAST;
  else if (dx == -1 && dy == 0) target = WEST;
  else if (dx == 0 && dy == 1) target = NORTH;
  else if (dx == 0 && dy == -1) target = SOUTH;

  int delta = (target - robotHeading + 4) % 4;
  // Use exact 90° increments so heading update is unambiguous
  if (delta == 1) { turn(110); }  // Turn Right
  else if (delta == 2) {
    turn(200);
  }                                    // Turn Around
  else if (delta == 3) { turn(-110); }  // Turn Left
  // if delta==0 do nothing

  moveForwardOneCellFast();


  robotX = nx;
  robotY = ny;
  visited[robotY][robotX] = true;
}
// =================================================================
// ================= ALGORITHM & UTILITY FUNCTIONS =================
// =================================================================

void IRAM_ATTR isrLeftEncoder() {
  encoderCountLeft++;
}
void IRAM_ATTR isrRightEncoder() {
  encoderCountRight++;
}

bool inBounds(int x, int y) {
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

void computeFloodfill(int gx, int gy) {
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++) distMap[y][x] = 10000;
  std::queue<std::pair<int, int>> q;
  distMap[gy][gx] = 0;
  q.push({ gx, gy });
  while (!q.empty()) {
    auto p = q.front();
    q.pop();
    int cx = p.first, cy = p.second;
    int d = distMap[cy][cx];
    if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy + 1) && distMap[cy + 1][cx] > d + 1) {
      distMap[cy + 1][cx] = d + 1;
      q.push({ cx, cy + 1 });
    }
    if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx + 1, cy) && distMap[cy][cx + 1] > d + 1) {
      distMap[cy][cx + 1] = d + 1;
      q.push({ cx + 1, cy });
    }
    if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy - 1) && distMap[cy - 1][cx] > d + 1) {
      distMap[cy - 1][cx] = d + 1;
      q.push({ cx, cy - 1 });
    }
    if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx - 1, cy) && distMap[cy][cx - 1] > d + 1) {
      distMap[cy][cx - 1] = d + 1;
      q.push({ cx - 1, cy });
    }
  }
}

// =================================================================
// ====================== *** CODE FIX *** =========================
// The original function had a logic error where it would lock onto
// the first "good" direction it found, ignoring other, equally
// valid paths. This new version correctly checks all directions
// before making a final decision.
// =================================================================
bool nextCellTowardsGoal(int &nx, int &ny) {
  int cx = robotX, cy = robotY;
  int bestD = distMap[cy][cx];
  int bestX = cx, bestY = cy;  // Start with current position as the "best"

  // Check North
  if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy + 1) && distMap[cy + 1][cx] < bestD) {
    bestD = distMap[cy + 1][cx];
    bestX = cx;
    bestY = cy + 1;
  }
  // Check East
  if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx + 1, cy) && distMap[cy][cx + 1] < bestD) {
    bestD = distMap[cy][cx + 1];
    bestX = cx + 1;
    bestY = cy;
  }
  // Check South
  if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy - 1) && distMap[cy - 1][cx] < bestD) {
    bestD = distMap[cy - 1][cx];
    bestX = cx;
    bestY = cy - 1;
  }
  // Check West
  if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx - 1, cy) && distMap[cy][cx - 1] < bestD) {
    bestD = distMap[cy][cx - 1];
    bestX = cx - 1;
    bestY = cy;
  }

  // If we found a better cell (i.e., best position is not the current one)
  if (bestX != cx || bestY != cy) {
    nx = bestX;
    ny = bestY;
    return true;
  }

  return false;  // No path forward found
}

// =================================================================
// ====================== *** CODE FIX *** =========================
// The pathReturnToStart function had the same logic error as
// nextCellTowardsGoal. This version is also corrected to properly
// find the best path back to the start.
// =================================================================
void pathReturnToStart() {
  while (!(robotX == 0 && robotY == 0)) {
    // Always sense new walls
    senseWallsAtCurrentCell();

    // Recompute floodfill distances with start (0,0) as goal
    computeFloodfill(0, 0);

    int nx = robotX, ny = robotY;
    if (nextCellTowardsGoal(nx, ny)) {
      executeMoveTo(nx, ny);
    } else {
      Serial.print("No path at ");
      Serial.print(robotX);
      Serial.print(", ");
      Serial.println(robotY);
      break;
    }

    delay(100);
  }

  // Ensure we really reached start
  if (robotX == 0 && robotY == 0) {
    robotHeading = NORTH;
    Serial.println("Returned to (0,0) successfully!");
    currentState = AWAITING_FAST_RUN;
  } else {
    Serial.print("Failed return, stuck at: ");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
  }
}


void runExploration() {
  // This is the same logic as our old loop()
  // Read both sensors using bus switching
  readAllSensors();

  Serial.print("ProxL: ");
  Serial.print(proxL);
  Serial.print(" | ProxR: ");
  Serial.println(proxR);
  senseWallsAtCurrentCell();
  computeFloodfill(goalX, goalY);

  if (robotX == goalX && robotY == goalY) {
    goalReached = true;
    stopMotors();
    Serial.println("Goal cell reached during exploration!");
    delay(1000);

    // =================================================================
    // ===================== *** SUGGESTION *** ========================
    // This logic was commented out. I've re-enabled it. After finding
    // the goal, the robot will now automatically find its way back
    // to the start and wait for your button press.
    // =================================================================
    Serial.println("Returning to start...");
    pathReturnToStart();
    Serial.println("Returned to start. Awaiting fast run command.");

    // Mission complete, change state and wait for button
    currentState = AWAITING_FAST_RUN;
    return;  // Exit this function
  }

  int nx = robotX, ny = robotY;
  bool hasNext = nextCellTowardsGoal(nx, ny);

  if (!hasNext) {
    // This happens if the robot is in a dead end.
    // The floodfill value of the current cell is higher than its neighbors,
    // but walls block the path. Re-computing the floodfill will
    // raise the "cost" of this dead end, forcing the robot to turn around.
    computeFloodfill(goalX, goalY);
    // After re-computing, try to find the next cell again.
    // This time it should point back the way it came.
    if (nextCellTowardsGoal(nx, ny)) {
      executeMoveTo(nx, ny);
    } else {
      // If it's still stuck, something is wrong. Maybe a 1x1 box.
      Serial.println("Error: Stuck in a loop!");
      turn(180);  // As a last resort, turn around.
    }
  } else {
    executeMoveTo(nx, ny);
  }
}
void performFastRun() {
  // Make sure the map is optimized for a path to the goal
  computeFloodfill(goalX, goalY);

  // Return to start position and heading before starting
  robotX = 0;
  robotY = 0;
  robotHeading = NORTH;  // Or whatever your start orientation is

  Serial.println("Executing fastest path to goal...");

  while (!(robotX == goalX && robotY == goalY)) {
    int cx = robotX, cy = robotY;
    int bestD = distMap[cy][cx];
    int nx = cx, ny = cy;

    // Find neighbor with the smallest distance value (greedy algorithm)
    if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy + 1) && distMap[cy + 1][cx] < bestD) {
      bestD = distMap[cy + 1][cx];
      nx = cx;
      ny = cy + 1;
    }
    if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx + 1, cy) && distMap[cy][cx + 1] < bestD) {
      bestD = distMap[cy][cx + 1];
      nx = cx + 1;
      ny = cy;
    }
    if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy - 1) && distMap[cy - 1][cx] < bestD) {
      bestD = distMap[cy - 1][cx];
      nx = cx;
      ny = cy - 1;
    }
    if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx - 1, cy) && distMap[cy][cx - 1] < bestD) {
      bestD = distMap[cy][cx - 1];
      nx = cx - 1;
      ny = cy;
    }

    if (nx == cx && ny == cy) {
      Serial.println("Stuck during fast run!");
      break;  // Should not happen with a complete map
    }

    // In a true speed run, you would use faster movement functions here
    executeMoveToFast(nx, ny);
  }

  Serial.println("Fast run complete! Goal reached!");
}