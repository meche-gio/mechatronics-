/*
 * M1V4.2 Self-Balancing Robot - State-Space Controller
 * Arduino Mega 2560 with MPU6050 and Encoders
 *
 * This version uses state-space feedback control with:
 * - Position feedback (from encoders)
 * - Velocity feedback (from encoders)
 * - Tilt angle feedback (from IMU)
 * - Tilt rate feedback (from gyro)
 *
 * Hardware Requirements:
 * - Arduino Mega 2560
 * - MPU6050 Gyroscope/Accelerometer
 * - L298N Motor Driver (or similar)
 * - Two DC Motors with encoders (360 PPR)
 * - 2S/3S LiPo Battery
 */

#include <Wire.h>
#include <Encoder.h>

// ================= TUNED CONTROL PARAMETERS =================

// State feedback gains - OPTIMIZED for better balance
float Kx        = 0.5f;       // position gain (m) - reduced for smoother response
float Kv        = 2.5f;       // velocity gain (m/s) - reduced to prevent overreaction
float Ktilt     = 1500.0f;    // tilt angle gain (rad) - REDUCED from 1900 to reduce oscillation
float KtiltRate = 200.0f;     // tilt rate gain (rad/s) - INCREASED from 160 for more damping

// PWM and safety limits
int   maxPWM       = 220;     // maximum PWM output
int   minPWM       = 40;      // minimum PWM to overcome friction
float deadzoneDeg  = 0.3f;    // reduce power near balance point (degrees)
float deadzoneGyro = 0.3f;    // gyro deadzone (deg/s)
float safetyDeg    = 35.0f;   // safety cutoff angle

// Angle trim offset - adjust this to calibrate your robot's "upright" position
// Positive value tilts the setpoint backward, negative tilts forward
float angleOffsetDeg = 0.08f;

// ================= MPU6050 REGISTERS =================

const uint8_t WHO_AM_I      = 0x68;
const uint8_t PWR_MGMT1     = 0x6B;
const uint8_t CONFIG        = 0x1A;
const uint8_t DLPF_CFG      = 0;
const uint8_t GYRO_CONFIG   = 0x1B;
const uint8_t GYRO_XOUT_H   = 0x43;
const uint8_t ACCEL_CONFIG  = 0x1C;
const uint8_t ACCEL_YOUT_H  = 0x3D;

// ================= ENCODER & MOTORS ==================

const int pinA = 2;
const int pinB = 3;
Encoder encoder(pinA, pinB);

// Motor pins - check your wiring!
const int motor1A = 5;
const int motor1B = 4;
const int motor2A = 11;
const int motor2B = 9;

// ================= CONSTANTS ========================

const float sampleTime   = 0.008f;  // 125 Hz control loop
const float pi           = 3.14159265359f;
const float wheelRadius  = 0.021f;  // meters (21mm) - adjust for your wheels
const float accWeight    = 0.04f;   // complementary filter weight (4% accelerometer)
const float accBias      = 0.0825f; // accelerometer angle bias (radians)
const float maxVoltage   = 3.25f;   // virtual max voltage for scaling

float gyroBias;
float conversionFactorGyro;
float conversionFactorAcc;

struct IIRFilter {
  float alpha;
  float prev = 0.0f;
};

// Sensor storage
float accValues[2];      // [0]=Ay, [1]=Az
float sensorValues[4];   // [0]=gyro, [1]=Ay, [2]=Az, [3]=encoder angle

// State variables
// [0]=position(m), [1]=velocity(m/s), [2]=tilt(rad), [3]=tiltRate(rad/s)
float states[4];

float referencePosition      = 0.0f;
float previousEncoderRadians = 0.0f;

IIRFilter accYFilter, accZFilter, velocityFilter;
unsigned long lastLoopTime = 0;

// ================= SETUP ============================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C

  Serial.println("M1V4.2 State-Space Balance Controller");
  Serial.println("Initializing...");

  // Wake up MPU6050
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  setupGyro();
  setupAcc();
  setupMotor();

  // Initialize filters
  initFilter(accYFilter, 0.2f);
  initFilter(accZFilter, 0.2f);
  initFilter(velocityFilter, 0.3f);

  Serial.println("Calibration complete!");
  Serial.println("Control gains:");
  Serial.print("  Kx (position): "); Serial.println(Kx);
  Serial.print("  Kv (velocity): "); Serial.println(Kv);
  Serial.print("  Ktilt (angle): "); Serial.println(Ktilt);
  Serial.print("  KtiltRate (damping): "); Serial.println(KtiltRate);
  Serial.println("\nReady to balance!");

  lastLoopTime = micros();
  delay(100);
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastLoopTime) / 1000000.0f;

  if (dt >= sampleTime) {
    lastLoopTime = currentTime;

    readSensors();
    updateStates(dt);
    float output = calculateOutput();
    setOutput(output);
    sendToGUI(output);
    receiveFromGUI();
  }
}

// ================= IMU SETUP ========================

void setupGyro() {
  // Configure gyro to ±250 deg/s
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Read back configuration
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(GYRO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)1);

  uint8_t val = Wire.read();
  uint8_t range = (val >> 3) & 0x03;

  switch (range) {
    case 0: conversionFactorGyro = 1.0f / 131.0f * pi / 180.0f; break;
    case 1: conversionFactorGyro = 1.0f / 65.5f * pi / 180.0f;  break;
    case 2: conversionFactorGyro = 1.0f / 32.8f * pi / 180.0f;  break;
    case 3: conversionFactorGyro = 1.0f / 16.4f * pi / 180.0f;  break;
  }

  // Calibrate gyro bias
  Serial.println("Calibrating gyro... keep still!");
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(WHO_AM_I);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(WHO_AM_I, (uint8_t)2);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    sum += raw;
    delay(1);
  }
  gyroBias = sum / 1000.0f;
  Serial.println("Gyro calibrated!");
}

void setupAcc() {
  // Configure accelerometer to ±2g
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Read back configuration
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(ACCEL_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)1);

  uint8_t val = Wire.read();
  uint8_t range = (val >> 3) & 0x03;

  switch (range) {
    case 0: conversionFactorAcc = 9.81567f / 16384.0f; break;
    case 1: conversionFactorAcc = 9.81567f / 8192.0f;  break;
    case 2: conversionFactorAcc = 9.81567f / 4096.0f;  break;
    case 3: conversionFactorAcc = 9.81567f / 2048.0f;  break;
  }
}

// ================= MOTOR SETUP ======================

void setupMotor() {
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  analogWrite(motor1A, 0);
  analogWrite(motor1B, 0);
  analogWrite(motor2A, 0);
  analogWrite(motor2B, 0);
}

// ================= SENSOR READING ===================

void readSensors() {
  float gyroValue    = readGyro();
  readAcc();
  float encoderValue = readEncoder();

  sensorValues[0] = gyroValue;
  sensorValues[1] = accValues[0];
  sensorValues[2] = accValues[1];
  sensorValues[3] = encoderValue;
}

float readGyro() {
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)2);

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return -(raw - gyroBias) * conversionFactorGyro; // rad/s
}

void readAcc() {
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)4);

  int16_t yRaw = (Wire.read() << 8) | Wire.read();
  int16_t zRaw = (Wire.read() << 8) | Wire.read();

  accValues[0] = yRaw * conversionFactorAcc * -1.0f; // Ay
  accValues[1] = zRaw * conversionFactorAcc;         // Az
}

float readEncoder() {
  long pulses = encoder.read();

  // If pushing robot forward makes wheels go wrong direction, uncomment:
  // pulses = -pulses;

  // Adjust 720.0f to your encoder PPR (pulses per revolution)
  return pulses * 2.0f * pi / 720.0f;
}

// ================= FILTER ===========================

void initFilter(IIRFilter &filter, float tau) {
  filter.alpha = sampleTime / (tau + sampleTime);
}

float lowPassFilter(float input, IIRFilter &filter) {
  filter.prev = filter.alpha * input + (1.0f - filter.alpha) * filter.prev;
  return filter.prev;
}

// ================= STATE ESTIMATION =================

void updateStates(float dt) {
  float gyroValue = sensorValues[0];
  float accY      = sensorValues[1];
  float accZ      = sensorValues[2];
  float enc       = sensorValues[3];

  // Filter accelerometer readings
  float fAccY = lowPassFilter(accY, accYFilter);
  float fAccZ = lowPassFilter(accZ, accZFilter);

  // Calculate tilt angle using complementary filter
  float tilt = calculateTiltAngle(gyroValue, fAccY, fAccZ, dt);

  // Calculate velocity from encoder
  float encVel = (enc - previousEncoderRadians) / dt;
  previousEncoderRadians = enc;

  float fEncVel = lowPassFilter(encVel, velocityFilter);

  // Update state variables
  float x    = enc * wheelRadius;      // position (m)
  float xDot = fEncVel * wheelRadius;  // velocity (m/s)

  states[0] = x - referencePosition;   // position error
  states[1] = xDot;                    // velocity
  states[2] = tilt;                    // tilt angle
  states[3] = gyroValue;               // tilt rate
}

float calculateTiltAngle(float gyro, float aY, float aZ, float dt) {
  static float angle = 0.0f;

  // Accelerometer angle
  float accAngle  = atan2(aZ, aY) - accBias;

  // Gyro integration
  float gyroDelta = -gyro * dt;

  // Complementary filter
  angle = (1.0f - accWeight) * (angle + gyroDelta) + accWeight * accAngle;

  // Apply trim offset
  float angleOffsetRad = angleOffsetDeg * pi / 180.0f;
  angle -= angleOffsetRad;

  return -angle;  // return tilt state
}

// ================= CONTROLLER =======================

float calculateOutput() {
  // State-space feedback: u = Kx*x + Kv*v + Ktilt*theta + KtiltRate*theta_dot
  return states[0] * Kx
       + states[1] * Kv
       + states[2] * Ktilt
       + states[3] * KtiltRate;
}

void setOutput(float output) {
  // Convert voltage to PWM
  float pwmFloat = fabs(output) / maxVoltage * (float)maxPWM;
  if (pwmFloat > maxPWM) pwmFloat = maxPWM;
  int pwm = (int)pwmFloat;

  float angleDeg = states[2] * 180.0f / pi;

  // Safety shutoff if tilted too far
  if (fabs(angleDeg) > safetyDeg) {
    analogWrite(motor1A, 0);
    analogWrite(motor1B, 0);
    analogWrite(motor2A, 0);
    analogWrite(motor2B, 0);
    return;
  }

  // Reduce power in deadzone near balance point
  if (fabs(angleDeg) < deadzoneDeg && fabs(states[3]) < deadzoneGyro) {
    pwm = pwm / 2;
  }

  // Apply minimum PWM to overcome static friction
  if (pwm > 0 && pwm < minPWM) {
    pwm = minPWM;
  }

  // Apply motor commands
  if (output > 0) {
    // Drive forward
    analogWrite(motor1A, 0);
    analogWrite(motor1B, pwm);
    analogWrite(motor2A, pwm);
    analogWrite(motor2B, 0);
  } else if (output < 0) {
    // Drive backward
    analogWrite(motor1A, pwm);
    analogWrite(motor1B, 0);
    analogWrite(motor2A, 0);
    analogWrite(motor2B, pwm);
  } else {
    analogWrite(motor1A, 0);
    analogWrite(motor1B, 0);
    analogWrite(motor2A, 0);
    analogWrite(motor2B, 0);
  }
}

// ================= SERIAL COMMUNICATION =============

void sendToGUI(float voltage) {
  // Clamp voltage for display
  float v = max(min(voltage, maxVoltage), -maxVoltage);

  // Send state data to serial plotter or GUI
  Serial.print("S");
  Serial.print(v, 4);                          // control voltage
  Serial.print(",");
  Serial.print(states[0], 4);                  // position error
  Serial.print(",");
  Serial.print(states[1], 4);                  // velocity
  Serial.print(",");
  Serial.print(states[0] + referencePosition, 4); // absolute position
  Serial.print(",");
  Serial.print(states[2], 4);                  // tilt angle
  Serial.print(",");
  Serial.println(referencePosition, 4);        // reference position
}

void receiveFromGUI() {
  // Receive reference position commands from serial
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      referencePosition = line.toFloat();
    }
  }
}
