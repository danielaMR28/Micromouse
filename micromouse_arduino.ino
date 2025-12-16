/*
 * Micromouse con giroscopio + Bluetooth + Encoders
 * Centrado automático y reversa INTELIGENTE hasta encontrar salida
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
#define ENCODER_A 3  // Motor izquierdo (interrupt pin)
#define ENCODER_B 2  // Motor derecho (interrupt pin)

// Sensores ultrasónicos
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_LEFT A0
#define ECHO_LEFT A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3

// MPU6050
const int MPU = 0x68;

// Constantes de velocidad
const int TURN_SPEED = 85;
const int SPEED_NORMAL = 80;
const int SPEED_REVERSE = 70;  // Velocidad para reversa (un poco más lento)
const int MOTOR_OFFSET = 5;

// Constantes de distancia (en cm)
const int FRONT_STOP = 7;
const int MAX_SIDE_DISTANCE = 12;
const int WALL_TOO_CLOSE = 4;
const int WALL_TOO_FAR = 8;
const int IDEAL_WALL_DISTANCE = 6;
const int BACK_WALL_DETECT = 7;  // Distancia para detectar pared atrás (si retrocedemos mucho)

// Constantes de giro y corrección
const int TURN_ANGLE = 38;
const float CORRECTION_FACTOR = 3.0;
const float WALL_CORRECTION_FACTOR = 8.0;

// Constantes de encoder y laberinto
const int PULSES_PER_CELL = 25;  // 25 pulsos = 17cm = 1 celda
const int MAX_REVERSE_CELLS = 6; // Máximo de celdas a retroceder (seguridad)

// Variables del giroscopio
float yaw = 0;
float gyroZ_offset = 0;
unsigned long lastTime;

// Variables de Bluetooth
unsigned long lastSend = 0;
const int SEND_INTERVAL = 100;

// Variables de encoders (volatile porque se usan en interrupciones)
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// Prototipos de funciones
void encoderISR_A();
void encoderISR_B();
void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight);
void driveForwardStraight();
bool escapeDeadEnd();
void driveBackwardOneCell();
void resetStraightDrive();
void turnRight90();
void turnLeft90();
void driveForward();
void updateGyro();
int getDistance(int trigPin, int echoPin);
void stopMotors();
void resetEncoders();
long getEncoderAverage();

void setup() {
  Serial.begin(9600);
  
  // Configurar pines de motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configurar pines de sensores
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Configurar encoders con interrupciones
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, RISING);
  
  // Inicializar MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  delay(2000);
  
  // Calibrar giroscopio
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
  
  delay(3000);
  Serial.println("START");
  resetStraightDrive();
}

void loop() {
  // Leer sensores
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Determinar presencia de paredes
  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  
  // Enviar datos por Bluetooth
  if (millis() - lastSend > SEND_INTERVAL) {
    Serial.print(distFront);
    Serial.print(",");
    Serial.print(distLeft);
    Serial.print(",");
    Serial.print(distRight);
    Serial.print(",");
    Serial.print(yaw, 1);
    Serial.print(",");
    
    if (wallFront && wallLeft && wallRight) {
      Serial.println("ATRAPADO");
    } else if (wallFront) {
      if (!wallRight) Serial.println("GIRO_DER");
      else if (!wallLeft) Serial.println("GIRO_IZQ");
      else Serial.println("MEDIA_VUELTA");
    } else {
      Serial.println("AVANZA");
    }
    lastSend = millis();
  }
  
  // Lógica de navegación
  if (wallFront) {
    stopMotors();
    delay(200);
    
    // Re-leer sensores laterales
    distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    
    // CASO ATRAPADO: Paredes en frente y ambos lados -> ESCAPE INTELIGENTE
    if (wallLeft && wallRight) {
      Serial.println(">>> INICIANDO ESCAPE <<<");
      escapeDeadEnd();
      resetStraightDrive();
    }
    // Casos normales de giro
    else if (!wallRight) {
      turnRight90();
      resetStraightDrive();
    } 
    else if (!wallLeft) {
      turnLeft90();
      resetStraightDrive();
    }
    else {
      // Media vuelta (no debería llegar aquí, pero por si acaso)
      turnLeft90();
      turnLeft90();
      resetStraightDrive();
    }
  }
  else {
    // Sin pared enfrente - avanzar con centrado
    driveForwardWithCentering(distLeft, distRight, wallLeft, wallRight);
  }
  
  delay(50);
}

/*
 * ESCAPE DE CALLEJÓN SIN SALIDA
 * Retrocede celda por celda hasta encontrar una salida lateral
 * Luego gira hacia esa salida
 */
bool escapeDeadEnd() {
  int cellsReversed = 0;
  bool foundExit = false;
  int exitDirection = 0;  // -1 = izquierda, 1 = derecha
  
  while (!foundExit && cellsReversed < MAX_REVERSE_CELLS) {
    // Retroceder una celda
    Serial.print("Reversa celda #");
    Serial.println(cellsReversed + 1);
    
    driveBackwardOneCell();
    cellsReversed++;
    
    // Detenerse y leer sensores
    stopMotors();
    delay(300);  // Esperar a que se estabilice
    
    int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
    
    bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);  // Ahora "frente" es atrás
    
    Serial.print("Dist L:");
    Serial.print(distLeft);
    Serial.print(" R:");
    Serial.print(distRight);
    Serial.print(" F:");
    Serial.println(distFront);
    
    // ¿Encontramos salida?
    if (!wallRight) {
      foundExit = true;
      exitDirection = 1;  // Salida a la derecha
      Serial.println(">>> SALIDA DERECHA <<<");
    }
    else if (!wallLeft) {
      foundExit = true;
      exitDirection = -1;  // Salida a la izquierda
      Serial.println(">>> SALIDA IZQUIERDA <<<");
    }
    
    // Seguridad: si hay pared muy cerca atrás, parar
    // (Nota: "frente" del sensor ahora apunta hacia donde vamos, o sea atrás del robot)
    // Si tienes sensor trasero, podrías usarlo aquí
  }
  
  // Realizar el giro hacia la salida
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
    // No encontró salida en MAX_REVERSE_CELLS, hacer media vuelta
    Serial.println("No se encontro salida, media vuelta");
    turnLeft90();
    turnLeft90();
    return false;
  }
}

/*
 * Retroceder exactamente una celda usando encoders
 */
void driveBackwardOneCell() {
  resetEncoders();
  
  // Resetear yaw para mantener línea recta en reversa
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    // Actualizar giroscopio para corrección
    updateGyro();
    
    // Corrección para ir recto en reversa (invertida)
    int correccion = yaw * CORRECTION_FACTOR;
    
    int velIzq = SPEED_REVERSE - correccion;  // Invertido porque vamos en reversa
    int velDer = (SPEED_REVERSE - MOTOR_OFFSET) + correccion;
    
    velIzq = constrain(velIzq, 50, 120);
    velDer = constrain(velDer, 50, 120);
    
    // Motores en reversa
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

// ISRs para encoders
void encoderISR_A() {
  encoderCountA++;
}

void encoderISR_B() {
  encoderCountB++;
}

// Resetear contadores de encoder
void resetEncoders() {
  noInterrupts();
  encoderCountA = 0;
  encoderCountB = 0;
  interrupts();
}

// Obtener promedio de pulsos de ambos encoders
long getEncoderAverage() {
  noInterrupts();
  long avg = (encoderCountA + encoderCountB) / 2;
  interrupts();
  return avg;
}

// Avanzar con centrado entre paredes
void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  // Corrección base por giroscopio
  int correccion = yaw * CORRECTION_FACTOR;
  
  // Corrección adicional por paredes laterales (centrado)
  int wallCorrection = 0;
  
  if (wallLeft && wallRight) {
    // Ambas paredes - centrar
    int diff = distLeft - distRight;
    wallCorrection = diff * WALL_CORRECTION_FACTOR / 2;
  }
  else if (wallLeft && distLeft < WALL_TOO_CLOSE) {
    // Muy cerca de pared izquierda
    wallCorrection = -(IDEAL_WALL_DISTANCE - distLeft) * WALL_CORRECTION_FACTOR;
  }
  else if (wallRight && distRight < WALL_TOO_CLOSE) {
    // Muy cerca de pared derecha
    wallCorrection = (IDEAL_WALL_DISTANCE - distRight) * WALL_CORRECTION_FACTOR;
  }
  else if (wallLeft && distLeft > WALL_TOO_FAR) {
    wallCorrection = (distLeft - IDEAL_WALL_DISTANCE) * WALL_CORRECTION_FACTOR / 2;
  }
  else if (wallRight && distRight > WALL_TOO_FAR) {
    wallCorrection = -(distRight - IDEAL_WALL_DISTANCE) * WALL_CORRECTION_FACTOR / 2;
  }
  
  wallCorrection = constrain(wallCorrection, -20, 20);
  
  int totalCorrection = correccion + wallCorrection;
  
  int velIzq = SPEED_NORMAL + totalCorrection;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - totalCorrection;
  
  velIzq = constrain(velIzq, 50, 150);
  velDer = constrain(velDer, 50, 150);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

void driveForwardStraight() {
  updateGyro();
  int correccion = yaw * CORRECTION_FACTOR;
  int velIzq = SPEED_NORMAL + correccion;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - correccion;
  velIzq = constrain(velIzq, 60, 150);
  velDer = constrain(velDer, 60, 150);
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
  resetEncoders();
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
  driveForward();
  delay(300);
  stopMotors();
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
  driveForward();
  delay(300);
  stopMotors();
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
  return duration * 0.034 / 2;
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
