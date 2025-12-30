/*
 * Micromouse: Floodfill Simulator (Final Version)
 * - Implements a state machine for exploration and speed runs.
 * - Runs entirely in the Serial Monitor.
 * - Set SIMULATION_MODE to 'false' to compile for real hardware.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <SparkFun_TB6612.h>
#include <queue>

// =================================================================
// SIMULATION FLAG: Set to 'true' for Serial Monitor simulation
// Set to 'false' to compile for the actual robot hardware
#define SIMULATION_MODE true
// =================================================================

// --- Pin Definitions (Used only when SIMULATION_MODE is false) ---
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 26
#define BIN2 27
#define PWMB 14
#define STBY_PIN 2
#define ENCL_A 39
#define ENCR_A 35
#define IR_FRONT_PIN 18
#define SDA_LEFT 23
#define SCL_LEFT 22
#define SDA_RIGHT 4
#define SCL_RIGHT 16
#define BUTTON_PIN 15

// --- Hardware Objects ---
#if !SIMULATION_MODE
Motor motorLeft = Motor(AIN1, AIN2, PWMA, 1, STBY_PIN);
Motor motorRight = Motor(BIN1, BIN2, PWMB, 1, STBY_PIN);
TwoWire I2C_Left = TwoWire(0);
TwoWire I2C_Right = TwoWire(1);
Adafruit_APDS9960 sensorLeft;
Adafruit_APDS9960 sensorRight;
#endif

// --- State Machine ---
enum ProgramState {
  EXPLORING,
  AWAITING_FAST_RUN,
  FAST_RUN,
  FINISHED
};
ProgramState currentState = EXPLORING;

// --- Robot Physical Constants ---
const float WHEEL_DIAMETER_CM   = 3.2;
const float WHEEL_SEPARATION_CM = 8.0;
const int   ENCODER_CPR         = 420;
const float CM_PER_TICK         = (PI * WHEEL_DIAMETER_CM) / ENCODER_CPR;
#define CELL_DISTANCE_CM 18.0

// --- Movement & Control Parameters ---
const int   BASE_SPEED          = 180;
const int   TURN_SPEED          = 140;
const float MOTOR_CORRECTION    = 1.0;
float Kp = 3.0;

// --- Sensor Calibration ---
const int LEFT_WALL_THRESHOLD  = 100;
const int RIGHT_WALL_THRESHOLD = 100;
const int FRONT_WALL_THRESHOLD = 2000;

// --- Global Variables ---
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

// --- Maze / Floodfill Data Structures ---
const int GRID_SIZE = 8;
int distMap[GRID_SIZE][GRID_SIZE];
uint8_t wallsMap[GRID_SIZE][GRID_SIZE];
bool visited[GRID_SIZE][GRID_SIZE];

#define WALL_N 1
#define WALL_E 2
#define WALL_S 4
#define WALL_W 8

enum Heading { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
int robotX = 0, robotY = 0;
Heading robotHeading = NORTH;

// --- Goal Coordinates ---
int goalX = GRID_SIZE/2 - 1, goalY = GRID_SIZE/2 - 1;

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
void runExploration();
void performFastRun();
void printMazeState();

#if !SIMULATION_MODE
int readLeftProx();
int readRightProx();
int readFrontSensor();
void IRAM_ATTR isrLeftEncoder();
void IRAM_ATTR isrRightEncoder();
#endif

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Micromouse Initializing...");

#if !SIMULATION_MODE
  attachHardware();
#else
  Serial.println("Mode: SIMULATION");
#endif

  for (int y=0; y<GRID_SIZE; y++){
    for (int x=0; x<GRID_SIZE; x++){
      wallsMap[y][x] = 0;
      distMap[y][x] = 10000;
      visited[y][x] = false;
    }
  }
  for (int y=0; y<GRID_SIZE; y++){ wallsMap[y][0] |= WALL_W; wallsMap[y][GRID_SIZE-1] |= WALL_E; }
  for (int x=0; x<GRID_SIZE; x++){ wallsMap[0][x] |= WALL_S; wallsMap[GRID_SIZE-1][x] |= WALL_N; }

  robotX = 0; robotY = 0; robotHeading = NORTH;
  visited[robotY][robotX] = true;

  computeFloodfill(goalX, goalY);

  Serial.println("State: EXPLORING. Ready to map the maze...");
  delay(1000);
}

// ------------------- Main loop -------------------
void loop() {
  switch (currentState) {
    case EXPLORING:
      runExploration();
      break;
    
    case AWAITING_FAST_RUN:
      if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 's' || command == 'S') {
          Serial.println("\n'S' command received! Starting FAST RUN!");
          currentState = FAST_RUN;
        }
      }
      break;
      
    case FAST_RUN:
      performFastRun();
      currentState = FINISHED;
      break;
      
    case FINISHED:
      // Do nothing. The simulation is complete.
      delay(1000);
      break;
  }
}

// =================================================================
// ================= STATE MACHINE & ALGORITHM =====================
// =================================================================

void runExploration() {
#if SIMULATION_MODE
  printMazeState();
#endif
  senseWallsAtCurrentCell();
  computeFloodfill(goalX, goalY);

  if (robotX == goalX && robotY == goalY) {
    stopMotors();
    Serial.println("Goal cell reached during exploration!");
#if SIMULATION_MODE
    printMazeState();
#endif
    delay(1000);
    computeFloodfill(0, 0);
    pathReturnToStart();
    Serial.println("Returned to start. Awaiting fast run command.");
    Serial.println("Type 'S' and press Enter to start the fast run.");
    currentState = AWAITING_FAST_RUN;
    return;
  }

  int nx = robotX, ny = robotY;
  bool hasNext = nextCellTowardsGoal(nx, ny);

  if (!hasNext) {
    turn(180);
  } else {
    executeMoveTo(nx, ny);
  }
#if SIMULATION_MODE
  delay(200);
#endif
}

void performFastRun() {
  computeFloodfill(goalX, goalY);
  robotX = 0;
  robotY = 0;
  robotHeading = NORTH;
  Serial.println("Executing fastest path to goal...");

  while (!(robotX == goalX && robotY == goalY)) {
#if SIMULATION_MODE
    printMazeState();
    delay(500);
#endif
    int cx = robotX, cy = robotY;
    int bestD = distMap[cy][cx];
    int nx = cx, ny = cy;
    if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy+1) && distMap[cy+1][cx] < bestD) { bestD = distMap[cy+1][cx]; nx = cx; ny = cy+1; }
    if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx+1, cy) && distMap[cy][cx+1] < bestD) { bestD = distMap[cy][cx+1]; nx = cx+1; ny = cy; }
    if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy-1) && distMap[cy-1][cx] < bestD) { bestD = distMap[cy-1][cx]; nx = cx; ny = cy-1; }
    if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx-1, cy) && distMap[cy][cx-1] < bestD) { bestD = distMap[cy][cx-1]; nx = cx-1; ny = cy; }

    if (nx == cx && ny == cy) {
      Serial.println("Stuck during fast run!");
      break;
    }
    executeMoveTo(nx, ny);
  }
  stopMotors();
#if SIMULATION_MODE
  printMazeState();
#endif
  Serial.println("Fast run complete! Goal reached!");
}

// =================================================================
// ================= SENSING & MOVEMENT ============================
// =================================================================

void senseWallsAtCurrentCell() {
  uint8_t cellWalls = wallsMap[robotY][robotX];
#if SIMULATION_MODE
  Serial.print("You are at ("); Serial.print(robotX); Serial.print(", "); Serial.print(robotY);
  Serial.println("). Which walls do you see?");
  Serial.println("Enter any combination of F, L, R (e.g., 'FR'), then press Enter:");

  while (Serial.available() == 0) { delay(50); }
  String wallInput = Serial.readStringUntil('\n');
  wallInput.toUpperCase();

  bool frontWall = wallInput.indexOf('F') != -1;
  bool leftWall = wallInput.indexOf('L') != -1;
  bool rightWall = wallInput.indexOf('R') != -1;

  Serial.print("Input recorded: ");
  if (frontWall) Serial.print("Front ");
  if (leftWall) Serial.print("Left ");
  if (rightWall) Serial.print("Right");
  Serial.println("");
#else
  delay(50);
  int leftVal = readLeftProx();
  int rightVal = readRightProx();
  int frontVal = readFrontSensor();
  Serial.print("Sensor values: L="); Serial.print(leftVal); Serial.print(" R="); Serial.print(rightVal); Serial.print(" F="); Serial.println(frontVal);
  bool leftWall = leftVal > LEFT_WALL_THRESHOLD;
  bool rightWall = rightVal > RIGHT_WALL_THRESHOLD;
  bool frontWall = frontVal > FRONT_WALL_THRESHOLD;
#endif

  if (leftWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_W; else if (robotHeading == EAST) cellWalls |= WALL_N; else if (robotHeading == SOUTH) cellWalls |= WALL_E; else cellWalls |= WALL_S;
  }
  if (rightWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_E; else if (robotHeading == EAST) cellWalls |= WALL_S; else if (robotHeading == SOUTH) cellWalls |= WALL_W; else cellWalls |= WALL_N;
  }
  if (frontWall) {
    if (robotHeading == NORTH) cellWalls |= WALL_N; else if (robotHeading == EAST) cellWalls |= WALL_E; else if (robotHeading == SOUTH) cellWalls |= WALL_S; else cellWalls |= WALL_W;
  }
  
  wallsMap[robotY][robotX] = cellWalls;

  if (cellWalls & WALL_N) if (inBounds(robotX, robotY + 1)) wallsMap[robotY + 1][robotX] |= WALL_S;
  if (cellWalls & WALL_E) if (inBounds(robotX + 1, robotY)) wallsMap[robotY][robotX + 1] |= WALL_W;
  if (cellWalls & WALL_S) if (inBounds(robotX, robotY - 1)) wallsMap[robotY - 1][robotX] |= WALL_N;
  if (cellWalls & WALL_W) if (inBounds(robotX - 1, robotY)) wallsMap[robotY][robotX - 1] |= WALL_E;
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
  if (delta == 1) turn(90);
  else if (delta == 2) turn(180);
  else if (delta == 3) turn(-90);

  moveForwardOneCell();

  robotX = nx;
  robotY = ny;
  visited[robotY][robotX] = true;
}

void moveForwardOneCell() {
#if SIMULATION_MODE
  Serial.println(">>> ACTION: Moving forward one cell.");
#else
  long targetTicks = CELL_DISTANCE_CM / CM_PER_TICK;
  noInterrupts();
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();
  while ((encoderCountLeft + encoderCountRight) / 2 < targetTicks) {
    long error = encoderCountLeft - encoderCountRight;
    float correction = Kp * error;
    setMotorSpeeds(BASE_SPEED - correction, BASE_SPEED + correction);
    delay(5);
  }
  stopMotors();
#endif
}

void turn(int degrees) {
#if SIMULATION_MODE
  if (degrees > 0) Serial.println(">>> ACTION: Turning right.");
  else if (degrees < 0) Serial.println(">>> ACTION: Turning left.");
  else Serial.println(">>> ACTION: Turning 180.");
#else
    float wheel_circumference = PI * WHEEL_DIAMETER_CM;
    float turn_circumference = PI * WHEEL_SEPARATION_CM;
    float distance_per_wheel = ( (float) abs(degrees) / 360.0) * turn_circumference;
    long targetTicks = distance_per_wheel / wheel_circumference * ENCODER_CPR;
    noInterrupts();
    encoderCountLeft = 0;
    encoderCountRight = 0;
    interrupts();
    if (degrees > 0) setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
    else setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    while ((abs(encoderCountLeft) + abs(encoderCountRight)) / 2 < targetTicks) {
        delay(5);
    }
    stopMotors();
#endif
    int turns = degrees / 90;
    robotHeading = (Heading)((robotHeading + turns + 4) % 4);
}

// =================================================================
// ====================== UTILITY FUNCTIONS ========================
// =================================================================

void printMazeState() {
  Serial.println();
  Serial.println("=================================================================");
  Serial.print("    Micromouse State | Pos: (");
  Serial.print(robotX); Serial.print(", "); Serial.print(robotY);
  Serial.print(") | Heading: ");
  if (robotHeading == NORTH) Serial.println("NORTH ^");
  if (robotHeading == EAST)  Serial.println("EAST  >");
  if (robotHeading == SOUTH) Serial.println("SOUTH v");
  if (robotHeading == WEST)  Serial.println("WEST  <");
  Serial.println("=================================================================");

  for (int y = GRID_SIZE - 1; y >= 0; y--) {
    Serial.print(y < 10 ? " " : ""); Serial.print(y); Serial.print(" +");
    for (int x = 0; x < GRID_SIZE; x++) {
      Serial.print((wallsMap[y][x] & WALL_N) ? "---" : "   ");
      Serial.print("+");
    }
    Serial.println();
    Serial.print("  |");
    for (int x = 0; x < GRID_SIZE; x++) {
      if (x == robotX && y == robotY) {
        Serial.print(" ");
        if (robotHeading == NORTH) Serial.print("^"); else if (robotHeading == EAST) Serial.print(">"); else if (robotHeading == SOUTH) Serial.print("v"); else if (robotHeading == WEST) Serial.print("<");
        Serial.print(" ");
      } else if (visited[y][x]) {
        Serial.print(" * ");
      } else {
        int d = distMap[y][x];
        if (d >= 1000) Serial.print("   "); else if (d > 99) Serial.print(d); else if (d > 9) { Serial.print(" "); Serial.print(d); Serial.print(" "); } else { Serial.print("  "); Serial.print(d); Serial.print(" "); }
      }
      Serial.print((wallsMap[y][x] & WALL_E) ? "|" : " ");
    }
    Serial.println();
  }
  Serial.print("  +");
  for (int x = 0; x < GRID_SIZE; x++) {
    Serial.print("---");
    Serial.print("+");
  }
  Serial.println();
  Serial.println("Legend: ^>v< Robot | * Visited Path | Numbers = Floodfill Distance");
  Serial.println();
}

void pathReturnToStart() {
  int cx = robotX, cy = robotY;
  while (!(cx == 0 && cy == 0)) {
    int bestD = distMap[cy][cx];
    int nx = cx, ny = cy;
    if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx,cy+1) && distMap[cy+1][cx] < bestD) { bestD = distMap[cy+1][cx]; nx = cx; ny = cy+1; }
    if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx+1,cy) && distMap[cy][cx+1] < bestD) { bestD = distMap[cy][cx+1]; nx = cx+1; ny = cy; }
    if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx,cy-1) && distMap[cy-1][cx] < bestD) { bestD = distMap[cy-1][cx]; nx = cx; ny = cy-1; }
    if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx-1,cy) && distMap[cy][cx-1] < bestD) { bestD = distMap[cy][cx-1]; nx = cx-1; ny = cy; }
    if (nx==cx && ny==cy) { Serial.println("Stuck during return path!"); break; }
    executeMoveTo(nx, ny);
    cx = nx; cy = ny;
#if SIMULATION_MODE
    printMazeState();
    delay(200);
#endif
  }
}

bool inBounds(int x, int y) { return x>=0 && x<GRID_SIZE && y>=0 && y<GRID_SIZE; }

void computeFloodfill(int gx, int gy) {
  for (int y=0; y<GRID_SIZE; y++) for (int x=0; x<GRID_SIZE; x++) distMap[y][x] = 10000;
  std::queue<std::pair<int,int>> q;
  distMap[gy][gx] = 0;
  q.push({gx,gy});
  while(!q.empty()) {
    auto p = q.front(); q.pop();
    int cx = p.first, cy = p.second;
    int d = distMap[cy][cx];
    if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy+1) && distMap[cy+1][cx] > d+1) { distMap[cy+1][cx] = d+1; q.push({cx, cy+1}); }
    if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx+1, cy) && distMap[cy][cx+1] > d+1) { distMap[cy][cx+1] = d+1; q.push({cx+1, cy}); }
    if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy-1) && distMap[cy-1][cx] > d+1) { distMap[cy-1][cx] = d+1; q.push({cx, cy-1}); }
    if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx-1, cy) && distMap[cy][cx-1] > d+1) { distMap[cy][cx-1] = d+1; q.push({cx-1, cy}); }
  }
}

bool nextCellTowardsGoal(int &nx, int &ny) {
  int cx = robotX, cy = robotY;
  int bestD = distMap[cy][cx];
  bool found = false;
  int temp_nx = nx, temp_ny = ny;
  if (!(wallsMap[cy][cx] & WALL_N) && inBounds(cx, cy+1) && distMap[cy+1][cx] < bestD) { bestD = distMap[cy+1][cx]; temp_nx = cx; temp_ny = cy+1; found = true; }
  if (!(wallsMap[cy][cx] & WALL_E) && inBounds(cx+1, cy) && distMap[cy][cx+1] < bestD) { bestD = distMap[cy][cx+1]; temp_nx = cx+1; temp_ny = cy; found = true; }
  if (!(wallsMap[cy][cx] & WALL_S) && inBounds(cx, cy-1) && distMap[cy-1][cx] < bestD) { bestD = distMap[cy-1][cx]; temp_nx = cx; temp_ny = cy-1; found = true; }
  if (!(wallsMap[cy][cx] & WALL_W) && inBounds(cx-1, cy) && distMap[cy][cx-1] < bestD) { bestD = distMap[cy][cx-1]; temp_nx = cx-1; temp_ny = cy; found = true; }
  if (found) { nx = temp_nx; ny = temp_ny; }
  return found;
}

// =================================================================
// ============== HARDWARE-ONLY FUNCTIONS & ISRs ===================
// =================================================================

#if !SIMULATION_MODE
void attachHardware() {
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  pinMode(IR_FRONT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  I2C_Left.begin(SDA_LEFT, SCL_LEFT, 400000);
  I2C_Right.begin(SDA_RIGHT, SCL_RIGHT, 400000);
  if (!sensorLeft.begin(10, APDS9960_AGAIN_1X, APDS9960_ADDRESS, &I2C_Left)) {
    Serial.println("Failed to initialize left APDS9960 sensor!");
  } else {
    Serial.println("Left APDS9960 initialized.");
    sensorLeft.setProxGain(APDS9960_PGAIN_8X);
    sensorLeft.enableProximity(true);
    sensorLeft.setProximityInterruptThreshold(0, 400);
    sensorLeft.enableProximityInterrupt();
  }
  if (!sensorRight.begin(10, APDS9960_AGAIN_1X, APDS9960_ADDRESS, &I2C_Right)) {
    Serial.println("Failed to initialize right APDS9960 sensor!");
  } else {
    Serial.println("Right APDS9960 initialized.");
    sensorRight.setProxGain(APDS9960_PGAIN_8X);
    sensorRight.enableProximity(true);
    sensorRight.setProximityInterruptThreshold(0, 400);
    sensorRight.enableProximityInterrupt();
  }
  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL_A), isrLeftEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), isrRightEncoder, FALLING);
  Serial.println("Hardware attached.");
}
void stopMotors() {
  motorLeft.brake();
  motorRight.brake();
}
int readLeftProx()  { return sensorLeft.readProximity(); }
int readRightProx() { return sensorRight.readProximity(); }
int readFrontSensor() { return analogRead(IR_FRONT_PIN); }
void IRAM_ATTR isrLeftEncoder()  { encoderCountLeft++; }
void IRAM_ATTR isrRightEncoder() { encoderCountRight++; }
#else
// Create empty stubs for hardware functions in simulation mode to prevent errors
void attachHardware() {}
void stopMotors() {}
#endif
