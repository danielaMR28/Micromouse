/*
* Micromouse con giroscopio + Bluetooth
* Desconectar pines 0 y 1 para subir código
*/
 
#include <Wire.h>
 
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 4
#define IN4 5
 
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_LEFT A0
#define ECHO_LEFT A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3
 
const int MPU = 0x68;
const int TURN_SPEED = 80;      // era 100
const int SPEED_NORMAL = 75;    // era 100
const int MOTOR_OFFSET = 5;
const int FRONT_STOP = 8;
const int MAX_SIDE_DISTANCE = 12;
 
const int TURN_ANGLE = 38;
const float CORRECTION_FACTOR = 3.5;  // era 3.0
 
float yaw = 0;
float gyroZ_offset = 0;
unsigned long lastTime;
unsigned long lastSend = 0;
const int SEND_INTERVAL = 100;
 
void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(2000);
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
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  // Enviar por Bluetooth cada SEND_INTERVAL ms
  if (millis() - lastSend > SEND_INTERVAL) {
    Serial.print(distFront);
    Serial.print(",");
    Serial.print(distLeft);
    Serial.print(",");
    Serial.print(distRight);
    Serial.print(",");
    Serial.print(yaw, 1);
    Serial.print(",");
    if (wallFront) {
      if (!wallRight) Serial.println("GIRO_DER");
      else if (!wallLeft) Serial.println("GIRO_IZQ");
      else Serial.println("MEDIA_VUELTA");
    } else {
      Serial.println("AVANZA");
    }
    lastSend = millis();
  }
  if (wallFront) {
    stopMotors();
    delay(200);
    distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
    wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
    wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
    if (!wallRight) {
      turnRight90();
      resetStraightDrive();
    } 
    else if (!wallLeft) {
      turnLeft90();
      resetStraightDrive();
    }
    else {
      turnLeft90();
      turnLeft90();
      resetStraightDrive();
    }
  }
  else {
    driveForwardStraight();
  }
  delay(50);
}
 
void driveForwardStraight() {
  updateGyro();
  int correccion = yaw * CORRECTION_FACTOR;
  int velIzq = SPEED_NORMAL + correccion;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - correccion;
  velIzq = constrain(velIzq, 50, 120);  // era 60, 150
  velDer = constrain(velDer, 50, 120);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
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
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}
