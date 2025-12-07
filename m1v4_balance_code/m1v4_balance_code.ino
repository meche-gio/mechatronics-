/*
 * M1V4.2 Self-Balancing Robot - Arduino Mega 2560
 * Two-wheeled balancing robot with PID control
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - MPU6050 Gyroscope/Accelerometer
 * - L298N Motor Driver (or similar)
 * - Two DC Motors with encoders
 * - 2S/3S LiPo Battery
 */

#include <Wire.h>
#include "config.h"

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// PID Constants - Tune these for your robot
double Kp = 40.0;   // Proportional gain
double Ki = 0.8;    // Integral gain
double Kd = 1.2;    // Derivative gain

// PID variables
double setpoint = 0.0;        // Target angle (0 = balanced)
double input = 0.0;           // Current angle
double output = 0.0;          // PID output
double lastError = 0.0;
double integral = 0.0;
unsigned long lastTime = 0;

// Motor control variables
int motorSpeedA = 0;
int motorSpeedB = 0;
int maxSpeed = 255;
int minSpeed = -255;

// Sensor variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float angleAcc, angleGyro;
float currentAngle = 0.0;
float gyroAngleX = 0.0;

// Timing
unsigned long timer = 0;
float dt = 0.01; // 10ms loop time

// Complementary filter constant
float alpha = 0.96;

// Dead zone - angle beyond which robot is considered fallen
const float FALL_ANGLE = 45.0;
bool isFallen = false;

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C

  // Initialize MPU6050
  initMPU6050();

  // Initialize motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);

  // Stop motors initially
  stopMotors();

  // Calibration delay
  Serial.println("M1V4.2 Balance Robot Initializing...");
  delay(1000);

  // Calibrate gyro
  calibrateGyro();

  Serial.println("Ready to balance!");
  timer = millis();
  lastTime = millis();
}

void loop() {
  // Calculate dt
  unsigned long currentTime = millis();
  dt = (currentTime - timer) / 1000.0;
  timer = currentTime;

  // Read sensors
  readMPU6050();

  // Calculate angle using complementary filter
  calculateAngle();

  // Check if robot has fallen
  if (abs(currentAngle) > FALL_ANGLE) {
    isFallen = true;
    stopMotors();
    Serial.println("Robot fallen! Restart to continue.");
    while(1) delay(1000); // Stop program
  }

  // Calculate PID
  input = currentAngle;
  output = calculatePID(input);

  // Apply motor control
  controlMotors(output);

  // Debug output (every 100ms)
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > 100) {
    Serial.print("Angle: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | PID Out: ");
    Serial.print(output, 2);
    Serial.print(" | Motor: ");
    Serial.println(motorSpeedA);
    lastPrint = currentTime;
  }

  // Maintain loop time (~10ms)
  while (millis() - currentTime < 10);
}

void initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up
  Wire.endTransmission(true);

  // Configure gyro range (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // ±250°/s
  Wire.endTransmission(true);

  // Configure accelerometer range (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // ±2g
  Wire.endTransmission(true);

  // Configure Digital Low Pass Filter
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // DLPF ~43Hz
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // Read accelerometer
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // Skip temperature
  Wire.read();
  Wire.read();

  // Read gyroscope
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Convert to meaningful units
  accX = ax / 16384.0; // ±2g range
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0; // ±250°/s range
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still!");
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    readMPU6050();
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(3);
  }

  // Store offsets (if needed, can be applied in readMPU6050)
  Serial.println("Calibration complete!");
}

void calculateAngle() {
  // Calculate angle from accelerometer
  angleAcc = atan2(accY, accZ) * RAD_TO_DEG;

  // Integrate gyro
  gyroAngleX += gyroX * dt;

  // Complementary filter
  currentAngle = alpha * (currentAngle + gyroX * dt) + (1 - alpha) * angleAcc;
}

double calculatePID(double input) {
  unsigned long now = millis();
  double timeChange = (now - lastTime) / 1000.0;

  if (timeChange <= 0) timeChange = 0.01;

  // Calculate error
  double error = setpoint - input;

  // Proportional term
  double pTerm = Kp * error;

  // Integral term with anti-windup
  integral += error * timeChange;
  integral = constrain(integral, -100, 100); // Anti-windup
  double iTerm = Ki * integral;

  // Derivative term
  double dTerm = Kd * (error - lastError) / timeChange;

  // Calculate total output
  double output = pTerm + iTerm + dTerm;

  // Remember for next iteration
  lastError = error;
  lastTime = now;

  return output;
}

void controlMotors(double pidOutput) {
  // Convert PID output to motor speed
  int speed = constrain((int)pidOutput, minSpeed, maxSpeed);

  motorSpeedA = speed;
  motorSpeedB = speed;

  // Motor A control
  if (motorSpeedA > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, abs(motorSpeedA));
  } else if (motorSpeedA < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_EN, abs(motorSpeedA));
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, 0);
  }

  // Motor B control
  if (motorSpeedB > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_EN, abs(motorSpeedB));
  } else if (motorSpeedB < 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_EN, abs(motorSpeedB));
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_EN, 0);
  }
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_EN, 0);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_EN, 0);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0; // Reset integral
}
