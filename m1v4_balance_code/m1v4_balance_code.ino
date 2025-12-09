/*
 * MinSeg M1V4.3 Self-Balancing Robot - Arduino Mega 2560
 * Two-wheeled balancing robot with PID control + HC-SR04 obstacle avoidance
 *
 * Hardware:
 * - Arduino Mega 2560
 * - MPU6050 Gyroscope/Accelerometer
 * - DRV8833 Dual Motor Driver
 * - HC-SR04 Ultrasonic Sensor
 * - Two DC Motors with encoders
 */

#include <Wire.h>
#include "config.h"

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// ===== PID CONSTANTS =====
// These values are tuned for a typical small self-balancing robot
// You may need to adjust based on your specific hardware
double Kp = DEFAULT_KP;   // Proportional - main balancing force
double Ki = DEFAULT_KI;   // Integral - corrects steady-state error
double Kd = DEFAULT_KD;   // Derivative - dampens oscillations

// PID variables
double setpoint = 0.0;        // Target angle (0 = balanced upright)
double input    = 0.0;        // Current angle from IMU
double output   = 0.0;        // PID output to motors
double lastError = 0.0;
double integral  = 0.0;
unsigned long lastTime = 0;

// Angle calibration
float angleOffset = 0.0f;     // Measured upright angle offset
unsigned long startTime = 0;  // For fall detection grace period

// Motor control
int motorSpeedA = 0;
int motorSpeedB = 0;

// IMU sensor data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float angleAcc, angleGyro;
float currentAngle = 0.0;
float gyroAngleX = 0.0;

// Encoder counts (optional feedback)
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// ===== HC-SR04 ULTRASONIC SENSOR =====
const int trigPin = TRIG_PIN;
const int echoPin = ECHO_PIN;
float distanceCM = 999.0;

// ===== OBSTACLE AVOIDANCE SETTINGS =====
// Distance thresholds (cm)
const float FAR_DIST      = 120.0;  // Beyond this: full forward
const float SLOW_DIST1    = 80.0;   // Start slowing
const float SLOW_DIST2    = 40.0;   // Slow more
const float REVERSE_DIST1 = 25.0;   // Start reversing
const float REVERSE_DIST2 = 15.0;   // Reverse harder

// Motion angles (degrees) - positive = forward, negative = backward
const float ANGLE_FORWARD_FAST   =  4.0;
const float ANGLE_FORWARD_SLOW   =  2.0;
const float ANGLE_ALMOST_STOP    =  0.5;
const float ANGLE_REVERSE_SOFT   = -1.5;
const float ANGLE_REVERSE_STRONG = -3.0;

// Smoothed motion setpoint
float motionAngle = 0.0;

// Timing
unsigned long timer = 0;
float dt = 0.01;  // Loop time in seconds

// Complementary filter weight (0.96 = trust gyro 96%, accel 4%)
const float alpha = 0.96;

// Fall detection
const float FALL_ANGLE = FALL_ANGLE_THRESHOLD;
bool isFallen = false;

// Dead band for motors (minimum PWM to overcome static friction)
const int MOTOR_DEADBAND = 25;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize I2C at 400kHz
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU6050
  initMPU6050();

  // Initialize DRV8833 motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_B, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_A), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), encoderBISR, CHANGE);

  // HC-SR04 pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  // Status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Ensure motors are stopped
  stopMotors();

  // Startup messages
  Serial.println(F("================================="));
  Serial.println(F("MinSeg M1V4.3 Balance Robot"));
  Serial.println(F("DRV8833 + MPU6050 + HC-SR04"));
  Serial.println(F("================================="));
  delay(500);

  // Calibrate gyroscope
  calibrateGyro();

  // Measure upright angle offset
  Serial.println(F("Hold robot upright and still..."));
  delay(1000);

  // Take multiple readings for stable offset
  float offsetSum = 0;
  for (int i = 0; i < 50; i++) {
    readMPU6050();
    angleAcc = atan2(accY, accZ) * RAD_TO_DEG;
    offsetSum += angleAcc;
    delay(10);
  }
  angleOffset = offsetSum / 50.0;

  Serial.print(F("Angle offset: "));
  Serial.println(angleOffset, 2);

  // Initialize angle tracking
  currentAngle = 0.0;
  gyroAngleX = 0.0;
  setpoint = 0.0;
  motionAngle = 0.0;

  Serial.println(F("Ready to balance!"));
  Serial.print(F("PID: Kp="));
  Serial.print(Kp);
  Serial.print(F(" Ki="));
  Serial.print(Ki);
  Serial.print(F(" Kd="));
  Serial.println(Kd);

  digitalWrite(LED_PIN, HIGH);

  timer = millis();
  lastTime = millis();
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Calculate time delta
  dt = (currentTime - timer) / 1000.0;
  if (dt <= 0.001) dt = 0.01;  // Minimum 1ms
  if (dt > 0.1) dt = 0.1;       // Maximum 100ms (prevents jumps)
  timer = currentTime;

  // Read IMU and calculate angle
  readMPU6050();
  calculateAngle();

  // Fall detection (with 2-second grace period at startup)
  if ((millis() - startTime > 2000) && (abs(currentAngle) > FALL_ANGLE)) {
    handleFall();
  }

  // Read ultrasonic sensor (every 50ms to avoid interference)
  static unsigned long lastSonarTime = 0;
  if (currentTime - lastSonarTime > 50) {
    distanceCM = readUltrasonicCM();
    lastSonarTime = currentTime;
  }

  // Calculate desired motion angle based on obstacle distance
  float desiredAngle = calculateMotionAngle();

  // Smooth the motion angle (low-pass filter)
  motionAngle = 0.85f * motionAngle + 0.15f * desiredAngle;

  // Set PID target
  setpoint = motionAngle;

  // Run PID controller
  input = currentAngle;
  output = calculatePID(input);

  // Apply to motors
  controlMotors(output);

  // Debug output (50Hz)
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 20) {
    printDebug();
    lastPrint = currentTime;
  }

  // Maintain consistent loop timing (~10ms / 100Hz)
  while (millis() - currentTime < 10);
}

// ================== IMU FUNCTIONS ==================

void initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission(true);
  delay(10);

  // Gyro range: ±250°/s (most sensitive)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);

  // Accelerometer range: ±2g (most sensitive)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);

  // Digital Low Pass Filter: ~44Hz bandwidth
  // This reduces noise but adds ~4.9ms delay
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);  // DLPF_CFG = 3
  Wire.endTransmission(true);

  delay(50);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  // Read accelerometer (6 bytes)
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Skip temperature (2 bytes)
  Wire.read();
  Wire.read();

  // Read gyroscope (6 bytes)
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Convert to physical units
  // ±2g range: 16384 LSB/g
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  // ±250°/s range: 131 LSB/(°/s), apply calibration offset
  gyroX = (gx / 131.0) - gyroXoffset;
  gyroY = (gy / 131.0) - gyroYoffset;
  gyroZ = (gz / 131.0) - gyroZoffset;
}

void calibrateGyro() {
  Serial.println(F("Calibrating gyro - keep robot still!"));

  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6, (uint8_t)true);

    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    sumX += gx / 131.0;
    sumY += gy / 131.0;
    sumZ += gz / 131.0;

    delay(3);
  }

  gyroXoffset = sumX / samples;
  gyroYoffset = sumY / samples;
  gyroZoffset = sumZ / samples;

  Serial.print(F("Gyro offsets - X:"));
  Serial.print(gyroXoffset, 3);
  Serial.print(F(" Y:"));
  Serial.print(gyroYoffset, 3);
  Serial.print(F(" Z:"));
  Serial.println(gyroZoffset, 3);
}

void calculateAngle() {
  // Angle from accelerometer (accurate but noisy)
  angleAcc = atan2(accY, accZ) * RAD_TO_DEG;

  // Angle from gyro integration (smooth but drifts)
  gyroAngleX += gyroX * dt;

  // Complementary filter: combine both
  // Trust gyro for fast changes, accelerometer for long-term accuracy
  currentAngle = alpha * (currentAngle + gyroX * dt) + (1.0 - alpha) * angleAcc;

  // Apply offset so upright = 0°
  currentAngle -= angleOffset;
}

// ================== PID CONTROLLER ==================

double calculatePID(double currentInput) {
  unsigned long now = millis();
  double timeChange = (now - lastTime) / 1000.0;

  if (timeChange <= 0.001) timeChange = 0.01;
  if (timeChange > 0.1) timeChange = 0.1;

  // Error calculation
  double error = setpoint - currentInput;

  // Proportional term
  double pTerm = Kp * error;

  // Integral term with anti-windup
  integral += error * timeChange;
  // Limit integral to prevent windup
  double integralLimit = 100.0 / Ki;  // Dynamic limit based on Ki
  if (integralLimit < 50) integralLimit = 50;
  if (integralLimit > 200) integralLimit = 200;
  integral = constrain(integral, -integralLimit, integralLimit);
  double iTerm = Ki * integral;

  // Derivative term (on error change)
  double derivative = (error - lastError) / timeChange;
  double dTerm = Kd * derivative;

  // Total PID output
  double pidOutput = pTerm + iTerm + dTerm;

  // Store for next iteration
  lastError = error;
  lastTime = now;

  return pidOutput;
}

// ================== MOTOR CONTROL ==================

void controlMotors(double pidOutput) {
  // Convert PID output to motor speed
  int speed = (int)pidOutput;

  // Constrain to valid PWM range
  speed = constrain(speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);

  // Direction correction (flip if robot runs away from balance point)
  // If robot falls forward when leaning forward, flip this sign
  speed = -speed;

  // Apply deadband compensation
  if (speed > 0 && speed < MOTOR_DEADBAND) {
    speed = MOTOR_DEADBAND;
  } else if (speed < 0 && speed > -MOTOR_DEADBAND) {
    speed = -MOTOR_DEADBAND;
  }

  motorSpeedA = speed;
  motorSpeedB = speed;

  // Drive motors using DRV8833 control scheme
  setMotorDRV8833(MOTOR_A_IN1, MOTOR_A_IN2, motorSpeedA);
  setMotorDRV8833(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeedB);
}

void setMotorDRV8833(int in1Pin, int in2Pin, int speed) {
  // DRV8833 control:
  // Forward:  IN1=PWM, IN2=LOW
  // Reverse:  IN1=LOW, IN2=PWM
  // Coast:    IN1=LOW, IN2=LOW
  // Brake:    IN1=HIGH, IN2=HIGH

  if (speed > 0) {
    analogWrite(in1Pin, speed);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    analogWrite(in2Pin, abs(speed));
  } else {
    // Coast stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0;  // Reset integral to prevent windup
}

void brakeMotors() {
  // Active brake (both pins HIGH)
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, HIGH);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0;
}

// ================== ENCODER ISRs ==================

void encoderAISR() {
  if (digitalRead(ENCODER_A_PIN_A) == digitalRead(ENCODER_A_PIN_B)) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

void encoderBISR() {
  if (digitalRead(ENCODER_B_PIN_A) == digitalRead(ENCODER_B_PIN_B)) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

// ================== ULTRASONIC SENSOR ==================

float readUltrasonicCM() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo with timeout (20ms max = ~340cm)
  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL);

  if (duration == 0) {
    return 999.0;  // No echo = no obstacle or too far
  }

  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = duration × 0.0343 / 2 (round trip)
  float distance = (duration * 0.0343f) / 2.0f;

  return distance;
}

// ================== OBSTACLE AVOIDANCE ==================

float calculateMotionAngle() {
  float desiredAngle;

  if (distanceCM == 999.0 || distanceCM > FAR_DIST) {
    // No obstacle: cruise forward
    desiredAngle = ANGLE_FORWARD_FAST;
  } else if (distanceCM > SLOW_DIST1) {
    // Far obstacle: slow forward
    desiredAngle = ANGLE_FORWARD_SLOW;
  } else if (distanceCM > SLOW_DIST2) {
    // Getting close: nearly stop
    desiredAngle = ANGLE_ALMOST_STOP;
  } else if (distanceCM > REVERSE_DIST1) {
    // Close: start reversing
    desiredAngle = ANGLE_REVERSE_SOFT;
  } else if (distanceCM > REVERSE_DIST2) {
    // Very close: reverse more
    desiredAngle = ANGLE_REVERSE_STRONG;
  } else {
    // Extremely close: strong reverse
    desiredAngle = ANGLE_REVERSE_STRONG * 1.5;
  }

  return desiredAngle;
}

// ================== FALL HANDLING ==================

void handleFall() {
  isFallen = true;
  stopMotors();
  digitalWrite(LED_PIN, LOW);

  Serial.println(F(""));
  Serial.println(F("=== ROBOT FALLEN ==="));
  Serial.print(F("Angle: "));
  Serial.println(currentAngle, 1);
  Serial.println(F("Reset Arduino to continue"));

  // Blink LED indefinitely
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(300);
  }
}

// ================== DEBUG OUTPUT ==================

void printDebug() {
  // Format for Arduino Serial Plotter compatibility
  Serial.print(F("angle:"));
  Serial.print(currentAngle, 1);
  Serial.print(F(" set:"));
  Serial.print(setpoint, 1);
  Serial.print(F(" pid:"));
  Serial.print(output, 0);
  Serial.print(F(" motor:"));
  Serial.print(motorSpeedA);
  Serial.print(F(" dist:"));
  Serial.println(distanceCM, 0);
}

// ================== UTILITY FUNCTIONS ==================

int readPotentiometer() {
  return analogRead(POT_PIN);
}

float mapPotToFloat(int potValue, float minVal, float maxVal) {
  return minVal + (maxVal - minVal) * (potValue / 1023.0);
}
