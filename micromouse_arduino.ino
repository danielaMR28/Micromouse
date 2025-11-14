// Micromouse con Arduino
// Control de motores con encoders, sensores ultrasónicos y comunicación Bluetooth

// === CONFIGURACIÓN DE PINES ===

// Motores (conectados al puente H)
#define MOTOR_LEFT_ENABLE 5   // PWM para velocidad motor izquierdo
#define MOTOR_LEFT_DIR1 4     // Dirección 1 motor izquierdo
#define MOTOR_LEFT_DIR2 3     // Dirección 2 motor izquierdo

#define MOTOR_RIGHT_ENABLE 6  // PWM para velocidad motor derecho
#define MOTOR_RIGHT_DIR1 7    // Dirección 1 motor derecho
#define MOTOR_RIGHT_DIR2 8    // Dirección 2 motor derecho

// Encoders
#define ENCODER_LEFT_A 2      // Pin interrupción encoder izquierdo
#define ENCODER_LEFT_B 9      // Pin B encoder izquierdo
#define ENCODER_RIGHT_A 3     // Pin interrupción encoder derecho
#define ENCODER_RIGHT_B 10    // Pin B encoder derecho

// Sensores ultrasónicos
#define TRIGGER_FRONT 11      // Trigger sensor frontal
#define ECHO_FRONT 12         // Echo sensor frontal
#define TRIGGER_LEFT A0       // Trigger sensor izquierdo
#define ECHO_LEFT A1          // Echo sensor izquierdo
#define TRIGGER_RIGHT A2      // Trigger sensor derecho
#define ECHO_RIGHT A3         // Echo sensor derecho

// === VARIABLES GLOBALES ===

// Control de motores
int baseSpeed = 150;          // Velocidad base (0-255)
int turnSpeed = 120;          // Velocidad para giros

// Encoders
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

// Distancias de sensores (en cm)
float distanceFront = 0;
float distanceLeft = 0;
float distanceRight = 0;

// Control de movimiento
const int PULSES_PER_CELL = 200;  // Pulsos del encoder por celda del laberinto
const int PULSES_PER_TURN = 100;  // Pulsos para un giro de 90 grados

// Comunicación
String inputCommand = "";
bool commandComplete = false;

// Estado del robot
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  READING_SENSORS
};

RobotState currentState = IDLE;

// === FUNCIONES DE CONFIGURACIÓN ===

void setup() {
  // Inicializar comunicación serial para Bluetooth
  Serial.begin(9600);
  
  // Configurar pines de motores
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Configurar pines de encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // Configurar interrupciones para encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, CHANGE);
  
  // Configurar pines de sensores ultrasónicos
  pinMode(TRIGGER_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIGGER_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIGGER_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Detener motores al inicio
  stopMotors();
  
  Serial.println("Micromouse iniciado");
  Serial.println("Comandos disponibles:");
  Serial.println("F - Avanzar una celda");
  Serial.println("L - Girar 90° izquierda");
  Serial.println("R - Girar 90° derecha");
  Serial.println("S - Leer sensores");
  Serial.println("H - Detener");
}

// === FUNCIONES DE INTERRUPCIÓN PARA ENCODERS ===

void encoderLeftISR() {
  // Leer el estado del pin B para determinar dirección
  if (digitalRead(ENCODER_LEFT_B) == digitalRead(ENCODER_LEFT_A)) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void encoderRightISR() {
  // Leer el estado del pin B para determinar dirección
  if (digitalRead(ENCODER_RIGHT_B) != digitalRead(ENCODER_RIGHT_A)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

// === FUNCIONES DE CONTROL DE MOTORES ===

void setMotorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_ENABLE, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_ENABLE, -speed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_ENABLE, 0);
  }
}

void setMotorRight(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_ENABLE, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_ENABLE, -speed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_ENABLE, 0);
  }
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// === FUNCIONES DE MOVIMIENTO ===

void moveForward() {
  // Reiniciar contadores de encoder
  encoderLeftCount = 0;
  encoderRightCount = 0;
  
  currentState = MOVING_FORWARD;
  
  // Mover hacia adelante hasta completar una celda
  while (abs(encoderLeftCount) < PULSES_PER_CELL && 
         abs(encoderRightCount) < PULSES_PER_CELL) {
    
    // Control PID simple para mantener recta la trayectoria
    int error = encoderLeftCount - encoderRightCount;
    int correction = error * 2; // Ganancia proporcional
    
    setMotorLeft(baseSpeed - correction);
    setMotorRight(baseSpeed + correction);
    
    // Verificar si hay obstáculos mientras avanza
    readSensors();
    if (distanceFront < 5) { // Si hay pared muy cerca
      stopMotors();
      break;
    }
    
    delay(10);
  }
  
  stopMotors();
  currentState = IDLE;
  
  Serial.println("MOVE_COMPLETE");
}

void turnLeft() {
  // Reiniciar contadores
  encoderLeftCount = 0;
  encoderRightCount = 0;
  
  currentState = TURNING_LEFT;
  
  // Girar rueda derecha hacia adelante, izquierda hacia atrás
  while (abs(encoderRightCount) < PULSES_PER_TURN) {
    setMotorLeft(-turnSpeed);
    setMotorRight(turnSpeed);
    delay(10);
  }
  
  stopMotors();
  currentState = IDLE;
  
  Serial.println("TURN_COMPLETE");
}

void turnRight() {
  // Reiniciar contadores
  encoderLeftCount = 0;
  encoderRightCount = 0;
  
  currentState = TURNING_RIGHT;
  
  // Girar rueda izquierda hacia adelante, derecha hacia atrás
  while (abs(encoderLeftCount) < PULSES_PER_TURN) {
    setMotorLeft(turnSpeed);
    setMotorRight(-turnSpeed);
    delay(10);
  }
  
  stopMotors();
  currentState = IDLE;
  
  Serial.println("TURN_COMPLETE");
}

// === FUNCIONES DE SENSORES ===

float readUltrasonic(int triggerPin, int echoPin) {
  // Generar pulso de trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // Leer duración del pulso echo
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout de 30ms
  
  // Calcular distancia en cm
  float distance = duration * 0.034 / 2;
  
  // Limitar a rango válido
  if (distance == 0 || distance > 400) {
    distance = 400;
  }
  
  return distance;
}

void readSensors() {
  distanceFront = readUltrasonic(TRIGGER_FRONT, ECHO_FRONT);
  distanceLeft = readUltrasonic(TRIGGER_LEFT, ECHO_LEFT);
  distanceRight = readUltrasonic(TRIGGER_RIGHT, ECHO_RIGHT);
}

void sendSensorData() {
  readSensors();
  
  // Convertir distancias a detección de pared (1 = pared, 0 = sin pared)
  int wallFront = (distanceFront < 15) ? 1 : 0;
  int wallLeft = (distanceLeft < 15) ? 1 : 0;
  int wallRight = (distanceRight < 15) ? 1 : 0;
  
  // Enviar datos en formato: SENSORS:left,front,right
  String sensorData = "SENSORS:" + String(wallLeft) + "," + 
                      String(wallFront) + "," + String(wallRight);
  Serial.println(sensorData);
  
  // También enviar distancias reales para debug
  String debugData = "DISTANCES:" + String(distanceLeft, 1) + "," + 
                     String(distanceFront, 1) + "," + String(distanceRight, 1);
  Serial.println(debugData);
}

// === FUNCIÓN PRINCIPAL ===

void loop() {
  // Procesar comandos recibidos por Bluetooth
  if (commandComplete) {
    processCommand();
    inputCommand = "";
    commandComplete = false;
  }
  
  // Leer datos del puerto serial (Bluetooth)
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      inputCommand += inChar;
    }
  }
  
  // Actualizar estado si es necesario
  if (currentState == IDLE) {
    // Enviar datos de sensores periódicamente cuando está inactivo
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 500) {
      sendSensorData();
      lastSensorUpdate = millis();
    }
  }
}

// === PROCESAMIENTO DE COMANDOS ===

void processCommand() {
  inputCommand.trim(); // Eliminar espacios en blanco
  
  if (inputCommand == "F") {
    Serial.println("Avanzando...");
    moveForward();
  }
  else if (inputCommand == "L") {
    Serial.println("Girando izquierda...");
    turnLeft();
  }
  else if (inputCommand == "R") {
    Serial.println("Girando derecha...");
    turnRight();
  }
  else if (inputCommand == "S") {
    Serial.println("Leyendo sensores...");
    sendSensorData();
  }
  else if (inputCommand == "H") {
    Serial.println("Deteniendo...");
    stopMotors();
    currentState = IDLE;
  }
  else if (inputCommand == "TEST") {
    // Modo de prueba
    testMotors();
  }
  else if (inputCommand.startsWith("SPEED:")) {
    // Ajustar velocidad: SPEED:150
    int newSpeed = inputCommand.substring(6).toInt();
    if (newSpeed >= 0 && newSpeed <= 255) {
      baseSpeed = newSpeed;
      Serial.println("Velocidad ajustada a: " + String(baseSpeed));
    }
  }
  else if (inputCommand == "STATUS") {
    // Enviar estado actual
    sendStatus();
  }
  else {
    Serial.println("Comando desconocido: " + inputCommand);
  }
}

// === FUNCIONES DE PRUEBA Y DEBUG ===

void testMotors() {
  Serial.println("=== PRUEBA DE MOTORES ===");
  
  // Probar motor izquierdo
  Serial.println("Motor izquierdo adelante...");
  setMotorLeft(150);
  delay(1000);
  setMotorLeft(0);
  delay(500);
  
  Serial.println("Motor izquierdo atrás...");
  setMotorLeft(-150);
  delay(1000);
  setMotorLeft(0);
  delay(500);
  
  // Probar motor derecho
  Serial.println("Motor derecho adelante...");
  setMotorRight(150);
  delay(1000);
  setMotorRight(0);
  delay(500);
  
  Serial.println("Motor derecho atrás...");
  setMotorRight(-150);
  delay(1000);
  setMotorRight(0);
  delay(500);
  
  // Probar ambos motores
  Serial.println("Ambos motores adelante...");
  setMotorLeft(150);
  setMotorRight(150);
  delay(1000);
  stopMotors();
  
  Serial.println("=== PRUEBA COMPLETADA ===");
}

void sendStatus() {
  String status = "STATUS:";
  
  // Estado actual
  switch(currentState) {
    case IDLE:
      status += "IDLE,";
      break;
    case MOVING_FORWARD:
      status += "MOVING,";
      break;
    case TURNING_LEFT:
      status += "TURN_L,";
      break;
    case TURNING_RIGHT:
      status += "TURN_R,";
      break;
    case READING_SENSORS:
      status += "READING,";
      break;
  }
  
  // Contadores de encoders
  status += "ENC_L:" + String(encoderLeftCount) + ",";
  status += "ENC_R:" + String(encoderRightCount) + ",";
  
  // Velocidad actual
  status += "SPEED:" + String(baseSpeed);
  
  Serial.println(status);
}

// === FUNCIONES AUXILIARES ===

void calibrateSensors() {
  Serial.println("=== CALIBRACIÓN DE SENSORES ===");
  
  for (int i = 0; i < 10; i++) {
    readSensors();
    Serial.print("Frente: ");
    Serial.print(distanceFront);
    Serial.print(" cm, Izquierda: ");
    Serial.print(distanceLeft);
    Serial.print(" cm, Derecha: ");
    Serial.print(distanceRight);
    Serial.println(" cm");
    delay(500);
  }
  
  Serial.println("=== CALIBRACIÓN COMPLETADA ===");
}

// Función para ajustar parámetros PID (opcional, para mejorar el control)
void tunePID() {
  // Esta función puede implementarse para ajustar los parámetros
  // de control PID para mejor seguimiento de línea recta
  // y giros más precisos
}
