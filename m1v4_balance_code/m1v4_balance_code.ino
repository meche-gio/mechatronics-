/*
 * MinSeg M1V4.3 Self-Balancing Robot - Arduino Mega 2560
 * Two-wheeled balancing robot with PID control
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - MinSeg M1V4.3 Shield with:
 *   - DRV8833 Motor Driver
 *   - MPU6050 Gyroscope/Accelerometer (I2C)
 *   - QMC5883 Compass (I2C, optional)
 *   - Lego NXT Motor with Encoder
 * - 9V Battery or appropriate power supply
 *
 * Pin Configuration (see config.h for details):
 * - Motor A: Pin 5 (PWM), Pin 4 (Direction)
 * - Motor B: Pin 6 (PWM), Pin 7 (Direction)
 * - Encoder A: Pins 2, 3
 * - Encoder B: Pins 18, 19
 * - I2C (IMU): Pins 20 (SDA), 21 (SCL)
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
int maxSpeed = MAX_PWM_VALUE;
int minSpeed = -MAX_PWM_VALUE;

// Sensor variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float angleAcc, angleGyro;
float currentAngle = 0.0;
float gyroAngleX = 0.0;

// Encoder variables (if using encoder feedback)
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// Timing
unsigned long timer = 0;
float dt = 0.01; // 10ms loop time

// Complementary filter constant
float alpha = 0.96;

// Dead zone - angle beyond which robot is considered fallen
const float FALL_ANGLE = FALL_ANGLE_THRESHOLD;
bool isFallen = false;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C

  // Initialize MPU6050
  initMPU6050();

  // Initialize motor pins for DRV8833
  // DRV8833 uses 2 pins per motor (no separate enable)
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

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Stop motors initially
  stopMotors();

  // Calibration delay
  Serial.println("MinSeg M1V4.3 Balance Robot Initializing...");
  Serial.println("Hardware: DRV8833 Motor Driver, MPU6050 IMU");
  delay(1000);

  // Calibrate gyro
  calibrateGyro();

  Serial.println("Ready to balance!");
  digitalWrite(LED_PIN, HIGH);  // LED on = ready
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
    digitalWrite(LED_PIN, LOW);
    Serial.println("Robot fallen! Restart to continue.");
    while(1) {
      // Blink LED to indicate fallen state
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }

  // Calculate PID
  input = currentAngle;
  output = calculatePID(input);

  // Apply motor control
  controlMotors(output);

  // Debug output (every 100ms)
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > DEBUG_INTERVAL) {
    Serial.print("Angle: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | PID Out: ");
    Serial.print(output, 2);
    Serial.print(" | Motor: ");
    Serial.print(motorSpeedA);
    Serial.print(" | Enc: ");
    Serial.println(encoderCountA);
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

  // Apply calibration offsets to gyro
  gyroX = (gx / 131.0) - gyroXoffset; // ±250°/s range
  gyroY = (gy / 131.0) - gyroYoffset;
  gyroZ = (gz / 131.0) - gyroZoffset;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still!");
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // Starting register for gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    sumX += gx / 131.0;
    sumY += gy / 131.0;
    sumZ += gz / 131.0;
    delay(3);
  }

  // Store offsets
  gyroXoffset = sumX / samples;
  gyroYoffset = sumY / samples;
  gyroZoffset = sumZ / samples;

  Serial.print("Gyro offsets - X: ");
  Serial.print(gyroXoffset);
  Serial.print(" Y: ");
  Serial.print(gyroYoffset);
  Serial.print(" Z: ");
  Serial.println(gyroZoffset);
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

/*
 * DRV8833 Motor Control
 *
 * The DRV8833 doesn't have a separate enable pin like the L298N.
 * Instead, it uses both input pins for direction AND speed control:
 *
 * IN1   IN2   Result
 * PWM   LOW   Forward at PWM speed
 * LOW   PWM   Reverse at PWM speed
 * LOW   LOW   Coast (motor stops gradually)
 * HIGH  HIGH  Brake (motor stops quickly)
 */
void controlMotors(double pidOutput) {
  // Convert PID output to motor speed
  int speed = constrain((int)pidOutput, minSpeed, maxSpeed);

  motorSpeedA = speed;
  motorSpeedB = speed;

  // Motor A control (DRV8833 style)
  setMotorDRV8833(MOTOR_A_IN1, MOTOR_A_IN2, motorSpeedA);

  // Motor B control (DRV8833 style)
  setMotorDRV8833(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeedB);
}

/*
 * Set motor speed using DRV8833 driver
 * pwmPin: The PWM-capable pin (IN1 or IN3)
 * dirPin: The direction pin (IN2 or IN4)
 * speed: -255 to 255 (negative = reverse)
 */
void setMotorDRV8833(int pwmPin, int dirPin, int speed) {
  if (speed > 0) {
    // Forward: PWM on IN1, LOW on IN2
    analogWrite(pwmPin, speed);
    digitalWrite(dirPin, LOW);
  } else if (speed < 0) {
    // Reverse: LOW on IN1, PWM on IN2
    digitalWrite(pwmPin, LOW);
    analogWrite(dirPin, abs(speed));
  } else {
    // Stop (coast)
    digitalWrite(pwmPin, LOW);
    digitalWrite(dirPin, LOW);
  }
}

void stopMotors() {
  // Coast stop - both pins LOW
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0; // Reset integral
}

void brakeMotors() {
  // Active brake - both pins HIGH
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, HIGH);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0;
}

// Encoder A interrupt service routine
void encoderAISR() {
  if (digitalRead(ENCODER_A_PIN_A) == digitalRead(ENCODER_A_PIN_B)) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

// Encoder B interrupt service routine
void encoderBISR() {
  if (digitalRead(ENCODER_B_PIN_A) == digitalRead(ENCODER_B_PIN_B)) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

// Utility function to read potentiometer (for tuning)
int readPotentiometer() {
  return analogRead(POT_PIN);
}

// Utility function to map pot value to PID parameter range
float mapPotToFloat(int potValue, float minVal, float maxVal) {
  return minVal + (maxVal - minVal) * (potValue / 1023.0);
}
