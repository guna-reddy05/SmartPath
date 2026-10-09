#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---------- TESTING MODE ----------
#define SIMULATION_ONLY false  

// ---------- CALIBRATION ----------
#define MOVE_TIME 800       
#define TURN_90_TIME 450    
#define TURN_180_TIME 900   
#define MOTOR_SPEED 180     
#define JERK_SPEED 230      

// ---------- GRID LIMITS ----------
#define MAX_ROW 60
#define MAX_COL 60
#define MAX_NODES (MAX_ROW * MAX_COL)

int numRows = 0, numCols = 0;

// ---------- MOTOR PINS ----------
#define IN1 27  
#define IN2 26  
#define IN3 25  
#define IN4 33  
#define ENA 14  
#define ENB 32  

// ---------- IR SENSORS (COMMENTED) ----------
// #define IR_LEFT 36
// #define IR_RIGHT 39
// #define IR_FRONT 34

// ---------- DATA STRUCTURES ----------
short goalN, curBotPos;
short oLN = 0, cLN = 0;
short *openList;
short *closedList;
bool (*obstacle)[MAX_COL];

struct Node {
  short g, h, f, parent; 
};
Node (*PF)[MAX_COL]; 

enum Direction {NORTH, EAST, SOUTH, WEST};
Direction botDir = NORTH;

// ---------- MOTOR CONTROL ----------
void setMotor(int left, int right) {
  if (SIMULATION_ONLY) return;

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  digitalWrite(IN1, left > 0);
  digitalWrite(IN2, left < 0);
  digitalWrite(IN3, right > 0);
  digitalWrite(IN4, right < 0);
}

void stopMotors() { 
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  setMotor(0, 0); 
  delay(150); 
}

// ---------- MOVEMENT ----------
void moveForward() {
  Serial.println("  [ACTION] Moving Forward...");
  
  if (SIMULATION_ONLY) { 
    delay(500); 
    return; 
  }
  
  unsigned long startTime = millis();
  while (millis() - startTime < MOVE_TIME) {

    // IR LOGIC DISABLED
    /*
    bool L = (digitalRead(IR_LEFT) == LOW);
    bool R = (digitalRead(IR_RIGHT) == LOW);
    bool F = (digitalRead(IR_FRONT) == LOW);

    if (F) { 
      Serial.println("  ! FRONT WALL DETECTED - Stopping early"); 
      break; 
    }

    if (L) { setMotor(JERK_SPEED, MOTOR_SPEED - 60); }
    else if (R) { setMotor(MOTOR_SPEED - 60, JERK_SPEED); }
    else { setMotor(MOTOR_SPEED, MOTOR_SPEED); }
    */

    setMotor(MOTOR_SPEED, MOTOR_SPEED);
    delay(15);
  }
  stopMotors();
}

void turnRight() { 
  Serial.println("  [ACTION] Turning RIGHT");
  setMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  delay(TURN_90_TIME); 
  stopMotors(); 
}

void turnLeft() { 
  Serial.println("  [ACTION] Turning LEFT");
  setMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  delay(TURN_90_TIME); 
  stopMotors(); 
}

void turnAround() { 
  Serial.println("  [ACTION] Turning 180°");
  setMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  delay(TURN_180_TIME); 
  stopMotors(); 
}

// ---------- A* LOGIC ----------
short H(short r, short c) { return abs(r - (goalN / numCols)) + abs(c - (goalN % numCols)); }

void addOpen(short node, short parent) {
  openList[oLN++] = node;
  short r = node / numCols; short c = node % numCols;
  PF[r][c].parent = parent;
  PF[r][c].g = (parent == -1) ? 0 : PF[parent/numCols][parent%numCols].g + 1;
  PF[r][c].h = H(r, c);
  PF[r][c].f = PF[r][c].g + PF[r][c].h;
}

short getBest() {
  short best = openList[0];
  short bestF = 32000; 
  for (short i = 0; i < oLN; i++) {
    short r = openList[i] / numCols; short c = openList[i] % numCols;
    if (PF[r][c].f < bestF) { bestF = PF[r][c].f; best = openList[i]; }
  }
  return best;
}

void removeOpen(short val) {
  for (short i = 0; i < oLN; i++) {
    if (openList[i] == val) {
      for (short j = i; j < oLN - 1; j++) openList[j] = openList[j + 1];
      oLN--; break;
    }
  }
}

void expand(short node) {
  short r = node / numCols; short c = node % numCols;
  short dr[4] = {-1, 1, 0, 0}; short dc[4] = {0, 0, -1, 1};
  for (int i = 0; i < 4; i++) {
    short nr = r + dr[i]; short nc = c + dc[i];
    if (nr >= 0 && nc >= 0 && nr < numRows && nc < numCols && !obstacle[nr][nc]) {
      short next = nr * numCols + nc;
      bool visited = false;
      for (short k = 0; k < cLN; k++) if (closedList[k] == next) visited = true;
      if (!visited) addOpen(next, node);
    }
  }
}

void runAStar() {
  Serial.println("\n[A*] Calculating path...");
  addOpen(curBotPos, -1); 
  while (oLN > 0) {
    short current = getBest();
    closedList[cLN++] = current;
    removeOpen(current);
    if (current == goalN) { Serial.println("[A*] Path Calculation Successful!"); return; }
    expand(current);
  }
}

// ---------- PATH EXECUTION ----------
void moveRobot() {
  static short path[MAX_NODES]; 
  short pathSize = 0;
  short curr = goalN;
  while (curr != -1) {
    path[pathSize++] = curr;
    curr = PF[curr / numCols][curr % numCols].parent;
  }

  for (int i = pathSize - 2; i >= 0; i--) {
    short nextN = path[i];
    short prevN = path[i+1];
    int diff = nextN - prevN;

    Direction req;
    if (diff == -numCols) req = NORTH;
    else if (diff == numCols) req = SOUTH;
    else if (diff == 1) req = EAST;
    else req = WEST;

    int turn = req - botDir;
    if (turn != 0) {
      if (turn == 1 || turn == -3) turnRight();
      else if (turn == -1 || turn == 3) turnLeft();
      else if (turn == 2 || turn == -2) turnAround();
    }
    botDir = req;
    moveForward();
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Maze_Bot");

  openList = (short*)malloc(MAX_NODES * sizeof(short));
  closedList = (short*)malloc(MAX_NODES * sizeof(short));
  obstacle = (bool (*)[MAX_COL])malloc(MAX_ROW * MAX_COL * sizeof(bool));
  PF = (Node (*)[MAX_COL])malloc(MAX_ROW * MAX_COL * sizeof(Node));

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  Serial.println("[SYSTEM] Setup Ready.");
}

// ---------- LOOP ----------
void loop() {
  resetForNewRun();
  readMatrix();
  runAStar();
  moveRobot();
  delay(5000);
}
