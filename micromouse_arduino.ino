/*
 * Micromouse con giroscopio + Bluetooth + Encoders
 * V3.1 - Sensores en tiempo real + soporte flood fill dinámico
 * 
 * Cambios V3.1:
 * - Sensores se envían SIEMPRE (en todos los estados)
 * - Mejor sincronización con Python
 * 
 * Comandos Bluetooth:
 * - GO: Iniciar mapeo
 * - STOP: Detener motores
 * - RECAL: Modo recalibración
 * - RECAL_DONE: Fin recalibración  
 * - RACE: Iniciar carrera
 * - PATH,n,x0,y0,...: Cargar ruta
 * - PARAMS,speed,turn_speed,turn_angle,kp,front,pulses,corr,race: Parámetros
 * - GET_PARAMS: Solicitar parámetros
 * 
 * Desconectar pines 0 y 1 para subir código
 */

#include <Wire.h>

// Motores
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 4
#define IN4 5

// Encoders
#define ENCODER_A 3
#define ENCODER_B 2

// Sensores ultrasónicos
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_LEFT A0
#define ECHO_LEFT A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3

// MPU6050
const int MPU = 0x68;

// ========== PARÁMETROS CALIBRABLES ==========
int TURN_SPEED = 85;
int SPEED_NORMAL = 80;
int SPEED_REVERSE = 70;
int SPEED_RACE = 100;
int MOTOR_OFFSET = 5;

int FRONT_STOP = 8;
int MAX_SIDE_DISTANCE = 14;
int MIN_SIDE_DISTANCE = 3;
int CENTER_DIST = 6;

int TURN_ANGLE = 38;
float CORRECTION_FACTOR = 3.0;
float KP_WALL = 1.8;
int CENTER_DEADZONE = 1;

int PULSES_PER_CELL = 25;
int MAX_REVERSE_CELLS = 6;

// ========== VARIABLES ==========
float yaw = 0;
float gyroZ_offset = 0;
unsigned long lastTime;
unsigned long lastSend = 0;
unsigned long lastSensorSend = 0;
const int SEND_INTERVAL = 150;
const int SENSOR_INTERVAL = 100;  // Sensores cada 100ms SIEMPRE

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

int cellX = 0;
int cellY = 0;
int direction = 0;
long lastCellPulses = 0;

// Estados
enum RobotState {
  STATE_WAITING,
  STATE_MAPPING,
  STATE_STOPPED,
  STATE_RECALIBRATE,
  STATE_RACE_READY,
  STATE_RACING
};

RobotState robotState = STATE_WAITING;

// Ruta para carrera
int racePath[100][2];
int racePathLength = 0;
int racePathIndex = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, RISING);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  delay(2000);
  calibrateGyro();
  delay(1000);
  
  Serial.println("READY");
  sendParams();
  yaw = 0;
  lastTime = micros();
}

void calibrateGyro() {
  float suma = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    suma += gz;
    delay(1);
  }
  gyroZ_offset = suma / 1000.0;
  Serial.println("GYRO_CAL_DONE");
}

void loop() {
  // Procesar comandos
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }
  
  // ========== ENVIAR SENSORES SIEMPRE ==========
  if (millis() - lastSensorSend > SENSOR_INTERVAL) {
    sendSensorsRealtime();
    lastSensorSend = millis();
  }
  
  // Máquina de estados
  switch (robotState) {
    case STATE_WAITING:
      if (millis() - lastSend > 500) {
        Serial.println("WAITING");
        lastSend = millis();
      }
      break;
      
    case STATE_MAPPING:
      handleMapping();
      break;
      
    case STATE_STOPPED:
      if (millis() - lastSend > 1000) {
        Serial.println("STATUS_STOPPED");
        lastSend = millis();
      }
      break;
      
    case STATE_RECALIBRATE:
      if (millis() - lastSend > 500) {
        Serial.println("STATUS_RECAL");
        lastSend = millis();
      }
      break;
      
    case STATE_RACE_READY:
      if (millis() - lastSend > 500) {
        Serial.println("STATUS_RACE_READY");
        lastSend = millis();
      }
      break;
      
    case STATE_RACING:
      handleRacing();
      break;
  }
  
  delay(10);
}

// ========== ENVÍO DE SENSORES EN TIEMPO REAL ==========
void sendSensorsRealtime() {
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  
  // Formato: SENSORS,distF,distL,distR,wallF,wallL,wallR,dir,yaw,cellX,cellY,state
  Serial.print("SENSORS,");
  Serial.print(distFront);
  Serial.print(",");
  Serial.print(distLeft);
  Serial.print(",");
  Serial.print(distRight);
  Serial.print(",");
  Serial.print(wallFront ? "1" : "0");
  Serial.print(",");
  Serial.print(wallLeft ? "1" : "0");
  Serial.print(",");
  Serial.print(wallRight ? "1" : "0");
  Serial.print(",");
  Serial.print(direction);
  Serial.print(",");
  Serial.print(yaw, 1);
  Serial.print(",");
  Serial.print(cellX);
  Serial.print(",");
  Serial.print(cellY);
  Serial.print(",");
  Serial.println(robotState);
}

void processCommand(String cmd) {
  if (cmd == "GO") {
    if (robotState == STATE_WAITING || robotState == STATE_STOPPED) {
      robotState = STATE_MAPPING;
      cellX = 0;
      cellY = 0;
      direction = 0;
      resetEncoders();
      lastCellPulses = 0;
      Serial.println("STARTING");
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
    robotState = STATE_STOPPED;
    Serial.println("STOPPED");
  }
  else if (cmd == "RECAL") {
    stopMotors();
    robotState = STATE_RECALIBRATE;
    Serial.println("RECAL_MODE");
  }
  else if (cmd == "RECAL_DONE") {
    robotState = STATE_RACE_READY;
    cellX = 0;
    cellY = 0;
    direction = 0;
    resetEncoders();
    lastCellPulses = 0;
    Serial.println("RACE_READY");
  }
  else if (cmd == "RACE") {
    if (robotState == STATE_RACE_READY && racePathLength > 0) {
      robotState = STATE_RACING;
      racePathIndex = 0;
      Serial.println("RACING");
    }
  }
  else if (cmd.startsWith("PATH,")) {
    parsePath(cmd);
  }
  else if (cmd.startsWith("PARAMS,")) {
    parseParams(cmd);
  }
  else if (cmd == "GET_PARAMS") {
    sendParams();
  }
  else if (cmd == "TURN_RIGHT") {
    turnRight90();
    Serial.println("TURN_DONE");
  }
  else if (cmd == "TURN_LEFT") {
    turnLeft90();
    Serial.println("TURN_DONE");
  }
  else if (cmd == "TURN_180") {
    turn180();
    Serial.println("TURN_DONE");
  }
  else if (cmd == "GYRO_RECAL") {
    stopMotors();
    calibrateGyro();
  }
  else if (cmd.startsWith("SET_DIR,")) {
    int newDir = cmd.substring(8).toInt();
    if (newDir >= 0 && newDir <= 3) {
      direction = newDir;
      Serial.print("DIR_SET,");
      Serial.println(direction);
    }
  }
  else if (cmd.startsWith("SET_POS,")) {
    int commaIdx = cmd.indexOf(',', 8);
    if (commaIdx > 0) {
      cellX = cmd.substring(8, commaIdx).toInt();
      cellY = cmd.substring(commaIdx + 1).toInt();
      Serial.print("POS_SET,");
      Serial.print(cellX);
      Serial.print(",");
      Serial.println(cellY);
    }
  }
}

void parsePath(String cmd) {
  int idx = 5;
  int commaIdx = cmd.indexOf(',', idx);
  if (commaIdx < 0) return;
  
  racePathLength = cmd.substring(idx, commaIdx).toInt();
  idx = commaIdx + 1;
  
  for (int i = 0; i < racePathLength && i < 100; i++) {
    commaIdx = cmd.indexOf(',', idx);
    if (commaIdx < 0) break;
    racePath[i][0] = cmd.substring(idx, commaIdx).toInt();
    idx = commaIdx + 1;
    
    commaIdx = cmd.indexOf(',', idx);
    if (commaIdx < 0) {
      racePath[i][1] = cmd.substring(idx).toInt();
    } else {
      racePath[i][1] = cmd.substring(idx, commaIdx).toInt();
      idx = commaIdx + 1;
    }
  }
  
  Serial.print("PATH_LOADED,");
  Serial.println(racePathLength);
}

void parseParams(String cmd) {
  int values[8];
  int idx = 7;
  int count = 0;
  
  while (idx < cmd.length() && count < 8) {
    int commaIdx = cmd.indexOf(',', idx);
    if (commaIdx < 0) {
      values[count] = cmd.substring(idx).toInt();
      count++;
      break;
    }
    values[count] = cmd.substring(idx, commaIdx).toInt();
    idx = commaIdx + 1;
    count++;
  }
  
  if (count >= 6) {
    SPEED_NORMAL = values[0];
    TURN_SPEED = values[1];
    TURN_ANGLE = values[2];
    KP_WALL = values[3] / 10.0;
    FRONT_STOP = values[4];
    PULSES_PER_CELL = values[5];
    if (count >= 7) CORRECTION_FACTOR = values[6] / 10.0;
    if (count >= 8) SPEED_RACE = values[7];
    
    Serial.println("PARAMS_SET");
    sendParams();
  }
}

void sendParams() {
  Serial.print("CURRENT_PARAMS,");
  Serial.print(SPEED_NORMAL);
  Serial.print(",");
  Serial.print(TURN_SPEED);
  Serial.print(",");
  Serial.print(TURN_ANGLE);
  Serial.print(",");
  Serial.print((int)(KP_WALL * 10));
  Serial.print(",");
  Serial.print(FRONT_STOP);
  Serial.print(",");
  Serial.print(PULSES_PER_CELL);
  Serial.print(",");
  Serial.print((int)(CORRECTION_FACTOR * 10));
  Serial.print(",");
  Serial.println(SPEED_RACE);
}

void handleMapping() {
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  
  // Enviar datos de mapeo (paredes detectadas)
  if (millis() - lastSend > SEND_INTERVAL) {
    sendMappingData(wallFront, wallLeft, wallRight);
    lastSend = millis();
  }
  
  checkCellTransition();
  
  if (wallFront) {
    stopMotors();
    delay(200);
    
    distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    if (wallLeft && wallRight) {
      Serial.println(">>> DEAD_END <<<");
      escapeDeadEnd();
      resetStraightDrive();
    }
    else if (!wallRight) {
      turnRight90();
      resetStraightDrive();
    } 
    else if (!wallLeft) {
      turnLeft90();
      resetStraightDrive();
    }
    else {
      turn180();
      resetStraightDrive();
    }
  }
  else {
    driveForwardWithCentering(distLeft, distRight, wallLeft, wallRight);
  }
}

void sendMappingData(bool wallFront, bool wallLeft, bool wallRight) {
  // Formato específico para mapeo: DATA,distF,distL,distR,wF,wL,wR,dir,yaw,x,y
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  Serial.print("DATA,");
  Serial.print(distFront);
  Serial.print(",");
  Serial.print(distLeft);
  Serial.print(",");
  Serial.print(distRight);
  Serial.print(",");
  Serial.print(wallFront ? "1" : "0");
  Serial.print(",");
  Serial.print(wallLeft ? "1" : "0");
  Serial.print(",");
  Serial.print(wallRight ? "1" : "0");
  Serial.print(",");
  Serial.print(direction);
  Serial.print(",");
  Serial.print(yaw, 1);
  Serial.print(",");
  Serial.print(cellX);
  Serial.print(",");
  Serial.println(cellY);
}

void handleRacing() {
  if (racePathIndex >= racePathLength) {
    stopMotors();
    robotState = STATE_STOPPED;
    Serial.println("RACE_COMPLETE");
    return;
  }
  
  int targetX = racePath[racePathIndex][0];
  int targetY = racePath[racePathIndex][1];
  
  int dx = targetX - cellX;
  int dy = targetY - cellY;
  
  int targetDir = -1;
  if (dy == -1) targetDir = 0;
  else if (dx == 1) targetDir = 1;
  else if (dy == 1) targetDir = 2;
  else if (dx == -1) targetDir = 3;
  
  if (targetDir >= 0 && targetDir != direction) {
    turnToDirection(targetDir);
  }
  
  driveForwardOneCell();
  
  cellX = targetX;
  cellY = targetY;
  racePathIndex++;
  
  Serial.print("RACE_CELL,");
  Serial.print(cellX);
  Serial.print(",");
  Serial.print(cellY);
  Serial.print(",");
  Serial.println(racePathIndex);
}

void turnToDirection(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  switch (diff) {
    case 1: turnRight90(); break;
    case 2: turn180(); break;
    case 3: turnLeft90(); break;
  }
}

void driveForwardOneCell() {
  resetEncoders();
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    if (distFront > 0 && distFront <= FRONT_STOP) {
      Serial.println("RACE_OBSTACLE");
      break;
    }
    
    int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    driveForwardWithCenteringRace(distLeft, distRight, wallLeft, wallRight);
    delay(5);
  }
  
  stopMotors();
  delay(100);
}

void driveForwardWithCenteringRace(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  float errorPared = 0;

  if (distLeft > 0 && distLeft < MIN_SIDE_DISTANCE) {
    errorPared = 12;
  } 
  else if (distRight > 0 && distRight < MIN_SIDE_DISTANCE) {
    errorPared = -12;
  }
  else if (wallLeft && wallRight) {
    errorPared = distRight - distLeft;
  } 
  else if (wallLeft) {
    errorPared = (CENTER_DIST - distLeft) * 1.5;
  }
  else if (wallRight) {
    errorPared = (distRight - CENTER_DIST) * 1.5;
  }

  if (abs(errorPared) < CENTER_DEADZONE) {
    errorPared = 0;
  }

  float corrParedLimitada = constrain(errorPared * KP_WALL, -40, 40);
  float corrGyro = yaw * CORRECTION_FACTOR;
  float correccionTotal = corrGyro + corrParedLimitada;
  
  int maxCorr = SPEED_RACE / 2;
  correccionTotal = constrain(correccionTotal, -maxCorr, maxCorr);
  
  int velIzq = SPEED_RACE + correccionTotal;
  int velDer = (SPEED_RACE - MOTOR_OFFSET) - correccionTotal;
  
  velIzq = constrain(velIzq, 50, 200);
  velDer = constrain(velDer, 50, 200);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

void checkCellTransition() {
  long currentPulses = getEncoderAverage();
  long pulsesInCell = currentPulses - lastCellPulses;
  
  if (pulsesInCell >= PULSES_PER_CELL) {
    int dx = 0, dy = 0;
    
    switch (direction) {
      case 0: dy = -1; break;
      case 1: dx = 1;  break;
      case 2: dy = 1;  break;
      case 3: dx = -1; break;
    }
    
    cellX += dx;
    cellY += dy;
    lastCellPulses = currentPulses;
    
    Serial.print("NEW_CELL,");
    Serial.print(cellX);
    Serial.print(",");
    Serial.print(cellY);
    Serial.print(",");
    Serial.println(direction);
  }
}

void updateCellPositionReverse() {
  int dx = 0, dy = 0;
  
  switch (direction) {
    case 0: dy = 1;  break;
    case 1: dx = -1; break;
    case 2: dy = -1; break;
    case 3: dx = 1;  break;
  }
  
  cellX += dx;
  cellY += dy;
  
  Serial.print("NEW_CELL,");
  Serial.print(cellX);
  Serial.print(",");
  Serial.print(cellY);
  Serial.print(",");
  Serial.println(direction);
}

bool escapeDeadEnd() {
  int cellsReversed = 0;
  bool foundExit = false;
  int exitDirection = 0;
  
  while (!foundExit && cellsReversed < MAX_REVERSE_CELLS) {
    Serial.print("REVERSE_CELL,");
    Serial.println(cellsReversed + 1);
    
    driveBackwardOneCell();
    cellsReversed++;
    
    updateCellPositionReverse();
    
    stopMotors();
    delay(300);
    
    int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    
    bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    if (!wallRight) {
      foundExit = true;
      exitDirection = 1;
      Serial.println("EXIT_RIGHT");
    }
    else if (!wallLeft) {
      foundExit = true;
      exitDirection = -1;
      Serial.println("EXIT_LEFT");
    }
  }
  
  if (foundExit) {
    if (exitDirection == 1) {
      turnRight90();
    } else {
      turnLeft90();
    }
    Serial.print("ESCAPE_DONE,");
    Serial.println(cellsReversed);
    return true;
  }
  else {
    Serial.println("ESCAPE_FAIL");
    turn180();
    return false;
  }
}

void driveBackwardOneCell() {
  resetEncoders();
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    updateGyro();
    
    int correccion = yaw * CORRECTION_FACTOR;
    int velIzq = SPEED_REVERSE - correccion;
    int velDer = (SPEED_REVERSE - MOTOR_OFFSET) + correccion;
    
    velIzq = constrain(velIzq, 50, 120);
    velDer = constrain(velDer, 50, 120);
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, velIzq);
    analogWrite(ENB, velDer);
    
    delay(5);
  }
  
  stopMotors();
}

void encoderISR_A() { encoderCountA++; }
void encoderISR_B() { encoderCountB++; }

void resetEncoders() {
  noInterrupts();
  encoderCountA = 0;
  encoderCountB = 0;
  interrupts();
  lastCellPulses = 0;
}

long getEncoderAverage() {
  noInterrupts();
  long avg = (encoderCountA + encoderCountB) / 2;
  interrupts();
  return avg;
}

void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  float errorPared = 0;

  if (distLeft > 0 && distLeft < MIN_SIDE_DISTANCE) {
    errorPared = 12;
  } 
  else if (distRight > 0 && distRight < MIN_SIDE_DISTANCE) {
    errorPared = -12;
  }
  else if (wallLeft && wallRight) {
    errorPared = distRight - distLeft;
  } 
  else if (wallLeft) {
    errorPared = (CENTER_DIST - distLeft) * 1.5;
  }
  else if (wallRight) {
    errorPared = (distRight - CENTER_DIST) * 1.5;
  }

  if (abs(errorPared) < CENTER_DEADZONE) {
    errorPared = 0;
  }

  float corrParedLimitada = constrain(errorPared * KP_WALL, -40, 40);
  float corrGyro = yaw * CORRECTION_FACTOR;
  float correccionTotal = corrGyro + corrParedLimitada;
  
  int maxCorr = SPEED_NORMAL / 2;
  correccionTotal = constrain(correccionTotal, -maxCorr, maxCorr);
  
  int velIzq = SPEED_NORMAL + correccionTotal;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - correccionTotal;
  
  velIzq = constrain(velIzq, 40, 180);
  velDer = constrain(velDer, 40, 180);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

void resetStraightDrive() {
  yaw = 0;
  lastTime = micros();
}

void turnRight90() {
  yaw = 0;
  lastTime = micros();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  
  while (yaw > -TURN_ANGLE) {
    updateGyro();
    delay(5);
  }
  
  stopMotors();
  delay(200);
  
  direction = (direction + 1) % 4;
  Serial.print("DIR_CHANGE,");
  Serial.println(direction);
  
  driveForward();
  delay(300);
  stopMotors();
  
  resetEncoders();
}

void turnLeft90() {
  yaw = 0;
  lastTime = micros();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  
  while (yaw < TURN_ANGLE) {
    updateGyro();
    delay(5);
  }
  
  stopMotors();
  delay(200);
  
  direction = (direction + 3) % 4;
  Serial.print("DIR_CHANGE,");
  Serial.println(direction);
  
  driveForward();
  delay(300);
  stopMotors();
  
  resetEncoders();
}

void turn180() {
  turnLeft90();
  turnLeft90();
}

void driveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL - MOTOR_OFFSET);
}

void updateGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  float gyroZ = (gz - gyroZ_offset) / 131.0;
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  yaw += gyroZ * dt;
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 15000);
  if (duration == 0) return 999;
  int d = duration * 0.034 / 2;
  return (d == 0) ? 999 : d;
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

