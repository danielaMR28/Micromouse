/*
 * Micromouse con giroscopio + Bluetooth + Encoders
 * V3.0 - Con calibración remota, modo carrera y STOP
 * * DESCRIPCIÓN GENERAL:
 * Este código controla un robot tipo Micromouse capaz de explorar un laberinto (Mapeo)
 * y recorrer una ruta óptima (Carrera). Utiliza PID para centrarse en las paredes
 * y un giroscopio para mantener el rumbo y realizar giros precisos.
 * * Comandos Bluetooth:
 * - GO: Iniciar mapeo (exploración)
 * - STOP: Detener motores inmediatamente
 * - RECAL: Entrar en modo recalibración (envía datos de sensores sin moverse)
 * - RECAL_DONE: Finalizar recalibración y prepararse para carrera
 * - RACE: Iniciar carrera (sigue la ruta cargada en racePath)
 * - PATH,n,x0,y0,...: Cargar ruta (n=longitud, pares de coordenadas x,y)
 * - PARAMS,...: Ajustar constantes PID y velocidades al vuelo
 * - GET_PARAMS: Ver los parámetros actuales
 * - TURN_RIGHT/LEFT/180: Comandos de prueba para ajustar giros
 * - GYRO_RECAL: Recalibrar el "cero" del giroscopio
 * * NOTA: Desconectar pines 0 (RX) y 1 (TX) al subir el código al Arduino.
 */

#include <Wire.h> // Librería para comunicación I2C (necesaria para el Giroscopio)

// ================= DEFINICIÓN DE PINES =================

// Motores (Driver L298N o similar)
// ENA/ENB: Control de velocidad (PWM)
// IN1-IN4: Control de dirección
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 4
#define IN4 5

// Encoders (Interrupciones hardware para contar vueltas de rueda)
#define ENCODER_A 3
#define ENCODER_B 2

// Sensores ultrasónicos (HC-SR04)
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_LEFT A0
#define ECHO_LEFT A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3

// Dirección I2C del sensor MPU6050
const int MPU = 0x68;

// ========== PARÁMETROS CALIBRABLES (AJUSTE FINO) ==========
// Velocidades PWM (0-255)
int TURN_SPEED = 85;      // Velocidad para girar sobre su propio eje
int SPEED_NORMAL = 80;    // Velocidad base de exploración
int SPEED_REVERSE = 70;   // Velocidad al retroceder en callejones sin salida
int SPEED_RACE = 100;     // Velocidad máxima durante la carrera rápida
int MOTOR_OFFSET = 5;     // Compensación si un motor es físicamente más rápido que el otro

// Distancias de umbral (en cm)
int FRONT_STOP = 8;        // Distancia a la pared frontal para detenerse
int MAX_SIDE_DISTANCE = 14; // Si la distancia es mayor, se asume que no hay pared
int MIN_SIDE_DISTANCE = 3;  // Distancia mínima de seguridad (muy pegado a la pared)
int CENTER_DIST = 6;        // Distancia ideal a la pared para mantenerse centrado

// Control y PID
int TURN_ANGLE = 38;         // Ángulo 'target' para giros (ajustar si gira más o menos de 90°)
float CORRECTION_FACTOR = 3.0; // Cuánto corrige el robot basado en el error del giroscopio
float KP_WALL = 1.8;           // Constante Proporcional: qué tan fuerte reacciona al acercarse a una pared
int CENTER_DEADZONE = 1;       // Zona muerta: si el error es pequeño, no corregir (evita oscilación)

// Odometría (medición de distancia por encoders)
int PULSES_PER_CELL = 25;    // Cuantos pulsos de encoder equivalen a avanzar una celda del laberinto
int MAX_REVERSE_CELLS = 6;   // Límite de celdas para retroceder antes de rendirse en un callejón

// ========== VARIABLES GLOBALES ==========
float yaw = 0;           // Ángulo actual de rotación (Z) acumulado
float gyroZ_offset = 0;  // Valor de calibración del giroscopio (drift)
unsigned long lastTime;  // Para calcular delta de tiempo (dt) en el giroscopio
unsigned long lastSend = 0; // Para controlar la frecuencia de envío de datos por Serial
const int SEND_INTERVAL = 150; // Enviar datos cada 150ms

// Contadores de encoders (volatile porque se usan en interrupciones)
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// Posición lógica en el laberinto
int cellX = 0;
int cellY = 0;
int direction = 0; // 0: Norte, 1: Este, 2: Sur, 3: Oeste
long lastCellPulses = 0;

// Estados del Robot (Máquina de Estados Finitos)
enum RobotState {
  STATE_WAITING,      // Esperando comando "GO"
  STATE_MAPPING,      // Explorando y resolviendo el laberinto
  STATE_STOPPED,      // Motores apagados
  STATE_RECALIBRATE,  // Modo ajuste de sensores
  STATE_RACE_READY,   // Ruta cargada, esperando "RACE"
  STATE_RACING        // Ejecutando la ruta rápida
};

RobotState robotState = STATE_WAITING;

// Almacenamiento de la ruta para la carrera
int racePath[100][2]; // Array para guardar hasta 100 coordenadas [x, y]
int racePathLength = 0;
int racePathIndex = 0;

// Prototipos de funciones
void encoderISR_A();
void encoderISR_B();
void processCommand(String cmd);
void sendParams();
void calibrateGyro();

// ================= SETUP =================
void setup() {
  Serial.begin(9600); // Comunicación Bluetooth/Serial
  
  // Configuración de pines de Motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configuración de pines de Sensores
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Configuración de Encoders con resistencia Pull-up interna
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Interrupciones: se activan cuando la señal del encoder sube (RISING)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, RISING);
  
  // Inicialización del MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Registro de gestión de energía
  Wire.write(0);    // Despertar el MPU
  Wire.endTransmission(true);
  
  delay(2000); // Esperar a que el robot esté quieto
  calibrateGyro(); // Calcular el error estático del giroscopio
  delay(1000);
  
  Serial.println("READY"); // Indicar a la app/consola que está listo
  sendParams(); // Enviar configuración actual
  yaw = 0;
  lastTime = micros();
}

// Calibra el giroscopio tomando 1000 muestras en reposo
void calibrateGyro() {
  float suma = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47); // Registro GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    suma += gz;
    delay(1);
  }
  gyroZ_offset = suma / 1000.0; // Guardar el promedio como offset
  Serial.println("GYRO_CAL_DONE");
}

// ================= LOOP PRINCIPAL =================
void loop() {
  // 1. Verificar si hay comandos entrantes por Bluetooth
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Eliminar espacios o saltos de línea extra
    processCommand(cmd);
  }
  
  // 2. Máquina de Estados: Ejecutar lógica según el estado actual
  switch (robotState) {
    case STATE_WAITING:
      // Solo envía latido de "espera"
      if (millis() - lastSend > 500) {
        Serial.println("WAITING");
        lastSend = millis();
      }
      break;
      
    case STATE_MAPPING:
      // Lógica principal de exploración
      handleMapping();
      break;
      
    case STATE_STOPPED:
      // Envía datos de sensores aunque esté parado (útil para debug)
      if (millis() - lastSend > 1000) {
        sendSensorData();
        lastSend = millis();
      }
      break;
      
    case STATE_RECALIBRATE:
      // Envío rápido de sensores para calibración visual en la app
      if (millis() - lastSend > 200) {
        sendRecalSensors();
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
      // Lógica de carrera rápida (Speed run)
      handleRacing();
      break;
  }
  
  delay(10); // Pequeña pausa para estabilidad
}

// Procesa los comandos de texto recibidos
void processCommand(String cmd) {
  if (cmd == "GO") {
    if (robotState == STATE_WAITING || robotState == STATE_STOPPED) {
      robotState = STATE_MAPPING;
      cellX = 0; cellY = 0; direction = 0;
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
    cellX = 0; cellY = 0; direction = 0;
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
  // Carga de ruta: PATH,longitud,x1,y1,x2,y2...
  else if (cmd.startsWith("PATH,")) {
    parsePath(cmd);
  }
  // Ajuste de constantes: PARAMS,val1,val2...
  else if (cmd.startsWith("PARAMS,")) {
    parseParams(cmd);
  }
  else if (cmd == "GET_PARAMS") {
    sendParams();
  }
  // Comandos manuales para probar giros
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
  // Forzar dirección lógica (útil si el robot se desorienta)
  else if (cmd.startsWith("SET_DIR,")) {
    int newDir = cmd.substring(8).toInt();
    if (newDir >= 0 && newDir <= 3) {
      direction = newDir;
      Serial.print("DIR_SET,");
      Serial.println(direction);
    }
  }
}

// Parsea la cadena de ruta recibida y llena el array racePath
void parsePath(String cmd) {
  int idx = 5;
  int commaIdx = cmd.indexOf(',', idx);
  if (commaIdx < 0) return;
  
  racePathLength = cmd.substring(idx, commaIdx).toInt();
  idx = commaIdx + 1;
  
  for (int i = 0; i < racePathLength && i < 100; i++) {
    // Parsea X
    commaIdx = cmd.indexOf(',', idx);
    if (commaIdx < 0) break;
    racePath[i][0] = cmd.substring(idx, commaIdx).toInt();
    idx = commaIdx + 1;
    
    // Parsea Y
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

// Parsea los parámetros de configuración PID y Velocidad
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
  
  // Asignar valores si llegaron suficientes datos
  if (count >= 6) {
    SPEED_NORMAL = values[0];
    TURN_SPEED = values[1];
    TURN_ANGLE = values[2];
    KP_WALL = values[3] / 10.0; // Se recibe como entero (ej: 18) y se pasa a float (1.8)
    FRONT_STOP = values[4];
    PULSES_PER_CELL = values[5];
    if (count >= 7) CORRECTION_FACTOR = values[6] / 10.0;
    if (count >= 8) SPEED_RACE = values[7];
    
    Serial.println("PARAMS_SET");
    sendParams();
  }
}

// Envía la configuración actual por Serial
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

void sendSensorData() {
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  Serial.print("SENSORS,");
  Serial.print(distFront);
  Serial.print(",");
  Serial.print(distLeft);
  Serial.print(",");
  Serial.println(distRight);
}

void sendRecalSensors() {
  // Función idéntica a sendSensorData pero con etiqueta distinta para la App
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  Serial.print("RECAL_SENSORS,");
  Serial.print(distFront);
  Serial.print(",");
  Serial.print(distLeft);
  Serial.print(",");
  Serial.println(distRight);
}

// ================= LÓGICA DE MAPEO (EXPLORACIÓN) =================
void handleMapping() {
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Detección booleana de muros basada en umbrales
  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  
  // Enviar telemetría periódica
  if (millis() - lastSend > SEND_INTERVAL) {
    sendData(wallFront, wallLeft, wallRight);
    lastSend = millis();
  }
  
  // Verificar si hemos avanzado una celda completa
  checkCellTransition();
  
  if (wallFront) {
    // Si hay pared en frente: Detener y decidir
    stopMotors();
    delay(200);
    
    // Re-leer sensores para decisión más estable
    distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    if (wallLeft && wallRight) {
      // Callejón sin salida: Iniciar rutina de escape (reversa)
      Serial.println(">>> INICIANDO ESCAPE <<<");
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
      // Caso raro: Solo pared frontal (T o Cruz), por defecto giro 180
      turn180();
      resetStraightDrive();
    }
  }
  else {
    // Si no hay pared frontal, avanzar centrando
    driveForwardWithCentering(distLeft, distRight, wallLeft, wallRight);
  }
}

// ================= LÓGICA DE CARRERA =================
void handleRacing() {
  // Si llegamos al final de la ruta
  if (racePathIndex >= racePathLength) {
    stopMotors();
    robotState = STATE_STOPPED;
    Serial.println("RACE_COMPLETE");
    return;
  }
  
  // Obtener siguiente celda objetivo del array
  int targetX = racePath[racePathIndex][0];
  int targetY = racePath[racePathIndex][1];
  
  // Calcular diferencias para saber dirección
  int dx = targetX - cellX;
  int dy = targetY - cellY;
  
  int targetDir = -1;
  if (dy == -1) targetDir = 0; // Norte
  else if (dx == 1) targetDir = 1; // Este
  else if (dy == 1) targetDir = 2; // Sur
  else if (dx == -1) targetDir = 3; // Oeste
  
  // Girar si es necesario para encarar la celda objetivo
  if (targetDir >= 0 && targetDir != direction) {
    turnToDirection(targetDir);
  }
  
  // Avanzar exactamente una celda usando encoders
  driveForwardOneCell();
  
  // Actualizar coordenadas lógicas
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

// Calcula el giro más corto hacia la dirección objetivo
void turnToDirection(int targetDir) {
  int diff = (targetDir - direction + 4) % 4;
  
  switch (diff) {
    case 1: turnRight90(); break;
    case 2: turn180(); break;
    case 3: turnLeft90(); break;
  }
}

// Avanza una cantidad fija de pulsos (una celda) en modo carrera
void driveForwardOneCell() {
  resetEncoders();
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    // Seguridad: Si aparece un obstáculo inesperado
    int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    if (distFront > 0 && distFront <= FRONT_STOP) {
      Serial.println("RACE_OBSTACLE");
      break;
    }
    
    int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    // Usa un algoritmo de centrado similar al mapeo pero optimizado para velocidad
    driveForwardWithCenteringRace(distLeft, distRight, wallLeft, wallRight);
    delay(5);
  }
  
  stopMotors();
  delay(100); // Pequeña pausa para inercia antes del siguiente paso
}

// Control PID específico para carrera (puede ser más agresivo)
void driveForwardWithCenteringRace(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  float errorPared = 0;

  // Prioridad 1: Evitar chocar si está MUY cerca de una pared (emergencia)
  if (distLeft > 0 && distLeft < MIN_SIDE_DISTANCE) {
    errorPared = 12; // Empujar fuerte a la derecha
  } 
  else if (distRight > 0 && distRight < MIN_SIDE_DISTANCE) {
    errorPared = -12; // Empujar fuerte a la izquierda
  }
  // Prioridad 2: Si hay dos paredes, centrarse en el medio
  else if (wallLeft && wallRight) {
    errorPared = distRight - distLeft;
  } 
  // Prioridad 3: Seguir una sola pared manteniendo CENTER_DIST
  else if (wallLeft) {
    errorPared = (CENTER_DIST - distLeft) * 1.5;
  }
  else if (wallRight) {
    errorPared = (distRight - CENTER_DIST) * 1.5;
  }

  // Zona muerta para evitar oscilaciones pequeñas
  if (abs(errorPared) < CENTER_DEADZONE) {
    errorPared = 0;
  }

  // Cálculo del PID (Solo P en este caso)
  float corrParedLimitada = constrain(errorPared * KP_WALL, -40, 40);
  float corrGyro = yaw * CORRECTION_FACTOR; // Usar giroscopio para mantener línea recta
  
  float correccionTotal = corrGyro + corrParedLimitada;
  
  // Limitar corrección máxima para no detener un motor por completo
  int maxCorr = SPEED_RACE / 2;
  correccionTotal = constrain(correccionTotal, -maxCorr, maxCorr);
  
  int velIzq = SPEED_RACE + correccionTotal;
  int velDer = (SPEED_RACE - MOTOR_OFFSET) - correccionTotal;
  
  // Saturación PWM
  velIzq = constrain(velIzq, 50, 200);
  velDer = constrain(velDer, 50, 200);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

// Actualiza las coordenadas X,Y basadas en los pulsos del encoder
void checkCellTransition() {
  long currentPulses = getEncoderAverage();
  long pulsesInCell = currentPulses - lastCellPulses;
  
  if (pulsesInCell >= PULSES_PER_CELL) {
    int dx = 0, dy = 0;
    
    // Actualizar coordenadas según dirección cardinal
    switch (direction) {
      case 0: dy = -1; break; // N
      case 1: dx = 1;  break; // E
      case 2: dy = 1;  break; // S
      case 3: dx = -1; break; // O
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

// Actualización de coordenadas cuando va en reversa
void updateCellPositionReverse() {
  int dx = 0, dy = 0;
  
  // La lógica es inversa a checkCellTransition
  switch (direction) {
    case 0: dy = 1;  break; // Si miro al N y voy atrás, Y aumenta
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

// Envío de telemetría completa
void sendData(bool wallFront, bool wallLeft, bool wallRight) {
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

// Maniobra para salir de callejones sin salida (Dead End)
bool escapeDeadEnd() {
  int cellsReversed = 0;
  bool foundExit = false;
  int exitDirection = 0;
  
  // Retrocede celda por celda hasta encontrar una apertura lateral
  while (!foundExit && cellsReversed < MAX_REVERSE_CELLS) {
    Serial.print("Reversa celda #");
    Serial.println(cellsReversed + 1);
    
    driveBackwardOneCell();
    cellsReversed++;
    
    updateCellPositionReverse();
    
    stopMotors();
    delay(300);
    
    // Verificar si se abrió un hueco a los lados
    int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    
    bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    sendData(false, wallLeft, wallRight);
    
    if (!wallRight) {
      foundExit = true;
      exitDirection = 1; // Salida a la derecha
      Serial.println(">>> SALIDA DERECHA <<<");
    }
    else if (!wallLeft) {
      foundExit = true;
      exitDirection = -1; // Salida a la izquierda
      Serial.println(">>> SALIDA IZQUIERDA <<<");
    }
  }
  
  if (foundExit) {
    if (exitDirection == 1) {
      turnRight90();
    } else {
      turnLeft90();
    }
    Serial.print("Escape completado en ");
    Serial.print(cellsReversed);
    Serial.println(" celdas");
    return true;
  }
  else {
    // Si llegamos al límite de reversa sin salida, dar media vuelta
    Serial.println("No se encontro salida, media vuelta");
    turn180();
    return false;
  }
}

// Función auxiliar para retroceder
void driveBackwardOneCell() {
  resetEncoders();
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    updateGyro();
    
    // Corrección inversa para mantener línea recta
    int correccion = yaw * CORRECTION_FACTOR;
    int velIzq = SPEED_REVERSE - correccion;
    int velDer = (SPEED_REVERSE - MOTOR_OFFSET) + correccion;
    
    velIzq = constrain(velIzq, 50, 120);
    velDer = constrain(velDer, 50, 120);
    
    // IN1/IN2 e IN3/IN4 invertidos para reversa
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

// Rutinas de Interrupción (ISR) para contar pulsos
void encoderISR_A() { encoderCountA++; }
void encoderISR_B() { encoderCountB++; }

void resetEncoders() {
  noInterrupts(); // Desactivar interrupciones para reset seguro
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

// CONTROL PRINCIPAL: PID DE PAREDES + GYRO (Modo Mapeo)
void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  float errorPared = 0;

  // Lógica de cálculo de error para centrado
  if (distLeft > 0 && distLeft < MIN_SIDE_DISTANCE) {
    errorPared = 12; // Muy cerca izq -> error positivo -> giro derecha
  } 
  else if (distRight > 0 && distRight < MIN_SIDE_DISTANCE) {
    errorPared = -12; // Muy cerca der -> error negativo -> giro izquierda
  }
  else if (wallLeft && wallRight) {
    errorPared = distRight - distLeft; // Centrado entre dos paredes
  } 
  else if (wallLeft) {
    errorPared = (CENTER_DIST - distLeft) * 1.5; // Seguir pared izq
  }
  else if (wallRight) {
    errorPared = (distRight - CENTER_DIST) * 1.5; // Seguir pared der
  }

  // Zona muerta
  if (abs(errorPared) < CENTER_DEADZONE) {
    errorPared = 0;
  }

  // Combinación de correcciones
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

// Giro de 90 grados a la derecha usando giroscopio
void turnRight90() {
  yaw = 0;
  lastTime = micros();
  // Configuración de giro sobre eje (motores en sentido opuesto)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  
  // Bucle de espera activa hasta alcanzar el ángulo
  while (yaw > -TURN_ANGLE) {
    updateGyro();
    delay(5);
  }
  
  stopMotors();
  delay(200);
  
  // Actualizar dirección lógica (0-3)
  direction = (direction + 1) % 4;
  Serial.print("DIR_CHANGE,");
  Serial.println(direction);
  
  // Pequeño empujón hacia adelante para realinearse
  driveForward();
  delay(300);
  stopMotors();
  
  resetEncoders();
}

// Giro de 90 grados a la izquierda
void turnLeft90() {
  yaw = 0;
  lastTime = micros();
  // Motores opuestos (Izquierda atrás, Derecha adelante)
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
  
  // Aritmética modular para restar dirección (equivalente a +3 mod 4)
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

// Movimiento simple sin PID (usado brevemente tras giros)
void driveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL - MOTOR_OFFSET);
}

// Lectura e integración del giroscopio
void updateGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47); // Registro GYRO_Z
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Restar offset y convertir a grados/segundo (LSB Sensitivity para 250dps es 131)
  float gyroZ = (gz - gyroZ_offset) / 131.0;
  
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0; // Tiempo delta en segundos
  lastTime = now;
  
  // Integración: Posición = Velocidad * Tiempo
  yaw += gyroZ * dt;
}

// Lectura de sensor ultrasónico con timeout
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 15000); // Timeout 15ms (~2.5m)
  if (duration == 0) return 999; // Retornar 999 si no hay rebote
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
