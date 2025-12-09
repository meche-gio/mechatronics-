/*
 * MinSeg M1V4.3 Self-Balancing Robot - Arduino Mega 2560
 * Two-wheeled balancing robot with PID control + HC-SR04 obstacle behavior
 *
 * Hardware:
 * - Arduino Mega 2560
 * - MinSeg Shield with DRV8833 Motor Driver
 * - MPU6050 Gyroscope/Accelerometer
 * - HC-SR04 Ultrasonic Sensor
 * - Two DC Motors with encoders
 */

#include <Wire.h>
#include "config.h"

// ================== MPU6050 ==================
#define MPU6050_ADDR 0x68

// ================== PID CONSTANTS ==================
double Kp = 180.0;    // Proportional gain
double Ki = 0.3;      // Integral gain
double Kd = 7.5;      // Derivative gain

// PID variables
double setpoint   = 0.0;   // Target angle (0 = upright)
double input      = 0.0;   // Current angle
double output     = 0.0;
double lastError  = 0.0;
double integral   = 0.0;
unsigned long lastTime = 0;

// Angle offset & fall detection
float angleOffset   = 0.0f;     // Upright angle offset
unsigned long startTime = 0;

// ================== Motor control ==================
int motorSpeedA = 0;
int motorSpeedB = 0;
int maxSpeed = MAX_PWM_VALUE;
int minSpeed = -MAX_PWM_VALUE;

// ================== IMU variables ==================
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float angleAcc;
float currentAngle = 0.0;
float gyroAngleX   = 0.0;

// ================== Encoders ==================
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// ================== Timing ==================
unsigned long timer = 0;
float dt    = 0.01;      // 10 ms loop time
float alpha = 0.96;      // complementary filter weight

// Fall detection
const float FALL_ANGLE = FALL_ANGLE_THRESHOLD;
bool isFallen = false;

// ================== HC-SR04 Ultrasonic ==================
// Using config.h pin definitions
const int TRIG_PIN = SONAR_TRIG_PIN;
const int ECHO_PIN = SONAR_ECHO_PIN;

float distanceCm       = 0.0;
float filteredDistance = 0.0;
unsigned long lastSonarTime = 0;
const unsigned long SONAR_INTERVAL = 50;   // ms between sonar pings

// Distance thresholds (cm)
const float MAX_VALID_DISTANCE = 200.0;  // ignore anything farther than 2 m

// Behavior bands (you can tune these)
const float FAR_DISTANCE   = 70.0;  // 70–150 cm: go forward
const float MID_DISTANCE   = 40.0;  // 40–70 cm: slower forward
const float NEAR_DISTANCE  = 25.0;  // 25–40 cm: almost stop
// <25 cm: reverse

// Drive command (in degrees of tilt)
float driveAngleCmd       = 0.0;   // desired tilt angle from sonar
float driveAngleCmdSmooth = 0.0;   // smoothed command (for smooth accel / decel)

// Max tilt angles (deg) for driving
const float MAX_DRIVE_ANGLE     = 3.0;   // forward tilt
const float MAX_REVERSE_ANGLE   = -3.0;  // backward tilt

// How fast we allow the drive angle to change each loop (deg per loop)
const float DRIVE_SLEW_PER_LOOP = 0.15;


// ================== Function Prototypes ==================
void initMPU6050();
void readMPU6050();
void calibrateGyro();
void calculateAngle();
double calculatePID(double input);
void controlMotors(double pidOutput);
void setMotorDRV8833(int in1Pin, int in2Pin, int speed);
void stopMotors();
void brakeMotors();

void encoderAISR();
void encoderBISR();

float measureDistanceCm();
void updateDriveCommand();

int readPotentiometer();
float mapPotToFloat(int potValue, float minVal, float maxVal);


// ================== SETUP ==================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  // I2C
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C

  // IMU
  initMPU6050();

  // Motors (pins defined in config.h)
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Encoders
  pinMode(ENCODER_A_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_A), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), encoderBISR, CHANGE);

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Stop motors initially
  stopMotors();

  Serial.println("MinSeg M1V4.3 Balance Robot Initializing...");
  Serial.println("Hardware: DRV8833 Motor Driver, MPU6050 IMU, HC-SR04 sonar");
  Serial.print("Motor A pins: IN1=");
  Serial.print(MOTOR_A_IN1);
  Serial.print(", IN2=");
  Serial.println(MOTOR_A_IN2);
  Serial.print("Motor B pins: IN1=");
  Serial.print(MOTOR_B_IN1);
  Serial.print(", IN2=");
  Serial.println(MOTOR_B_IN2);
  Serial.print("Sonar pins: TRIG=");
  Serial.print(TRIG_PIN);
  Serial.print(", ECHO=");
  Serial.println(ECHO_PIN);
  delay(1000);

  // Gyro calibration
  calibrateGyro();
  Serial.println("Measuring upright angle offset... Keep robot upright & still");

  delay(500);          // time to hold upright
  readMPU6050();
  timer = millis();
  dt = 0.01;
  calculateAngle();
  angleOffset = currentAngle;

  Serial.print("Angle offset (upright) = ");
  Serial.println(angleOffset, 2);

  // We want 0° = upright
  setpoint = 0.0;

  Serial.println("Ready to balance!");
  digitalWrite(LED_PIN, HIGH);

  timer        = millis();
  lastTime     = millis();
  startTime    = millis();
  lastSonarTime = millis();
}


// ================== LOOP ==================
void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - timer) / 1000.0;
  if (dt <= 0) dt = 0.01;
  timer = currentTime;

  // --- IMU update & angle estimation ---
  readMPU6050();
  calculateAngle();

  // --- Fall detection (after first 2 s) ---
  if ((millis() - startTime > 2000) && (abs(currentAngle) > FALL_ANGLE)) {
    isFallen = true;
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    Serial.println("Robot fallen! Restart to continue.");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }

  // --- Ultrasonic distance & drive command ---
  if (currentTime - lastSonarTime >= SONAR_INTERVAL) {
    distanceCm = measureDistanceCm();

    // Simple low-pass filter on distance
    if (distanceCm > 0 && distanceCm < MAX_VALID_DISTANCE) {
      if (filteredDistance == 0.0) {
        filteredDistance = distanceCm;
      } else {
        filteredDistance = 0.7f * filteredDistance + 0.3f * distanceCm;
      }
    }

    updateDriveCommand();  // sets driveAngleCmd & driveAngleCmdSmooth
    lastSonarTime = currentTime;
  }

  // The balancing setpoint is shifted by the sonar drive angle command
  setpoint = driveAngleCmdSmooth;

  // --- PID balance control ---
  input  = currentAngle;
  output = calculatePID(input);

  // --- Apply motor control ---
  controlMotors(output);

  // --- Debug output (every DEBUG_INTERVAL ms) ---
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint > DEBUG_INTERVAL) {
    Serial.print("Angle: ");
    Serial.print(currentAngle, 2);
    Serial.print("  | SP: ");
    Serial.print(setpoint, 2);
    Serial.print("  | PID: ");
    Serial.print(output, 2);

    Serial.print("  | DistRaw: ");
    Serial.print(distanceCm, 1);
    Serial.print("  | DistFilt: ");
    Serial.print(filteredDistance, 1);

    Serial.print("  | DriveCmd: ");
    Serial.print(driveAngleCmdSmooth, 2);

    Serial.print("  | MotorA: ");
    Serial.print(motorSpeedA);
    Serial.print("  | EncA: ");
    Serial.println(encoderCountA);

    lastPrint = currentTime;
  }

  // Maintain loop time (~10 ms)
  while (millis() - currentTime < 10);
}


// ================== MPU6050 FUNCTIONS ==================
void initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Gyro ±250°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accel ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  // DLPF ~43 Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // starting register: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read(); // ACCEL_XOUT
  int16_t ay = Wire.read() << 8 | Wire.read(); // ACCEL_YOUT
  int16_t az = Wire.read() << 8 | Wire.read(); // ACCEL_ZOUT

  Wire.read();  // TEMP_OUT_H (ignored)
  Wire.read();  // TEMP_OUT_L (ignored)

  int16_t gx = Wire.read() << 8 | Wire.read(); // GYRO_XOUT
  int16_t gy = Wire.read() << 8 | Wire.read(); // GYRO_YOUT
  int16_t gz = Wire.read() << 8 | Wire.read(); // GYRO_ZOUT

  // Convert to g's
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  // IMPORTANT:
  // Use GYRO Y as our "pitch" rate (forward/backward tilt)
  // and store it in gyroX so the rest of the code uses gyroX consistently.
  gyroX = (gy / 131.0) - gyroXoffset;  // pitch rate
  gyroY = (gx / 131.0) - gyroYoffset;  // roll rate (unused)
  gyroZ = (gz / 131.0) - gyroZoffset;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still!");
  float sumPitch = 0;
  float sumRoll  = 0;
  float sumZ     = 0;
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    // gy is our pitch axis
    sumPitch += gy / 131.0;
    sumRoll  += gx / 131.0;
    sumZ     += gz / 131.0;

    delay(3);
  }

  gyroXoffset = sumPitch / samples;  // pitch offset
  gyroYoffset = sumRoll  / samples;  // roll offset
  gyroZoffset = sumZ     / samples;  // yaw offset

  Serial.print("Gyro offsets - Pitch(X): ");
  Serial.print(gyroXoffset);
  Serial.print("  Roll(Y): ");
  Serial.print(gyroYoffset);
  Serial.print("  Z: ");
  Serial.println(gyroZoffset);
  Serial.println("Calibration complete!");
}

void calculateAngle() {
  // Use X–Z plane for pitch (forward/back): tilt changes accX strongly
  angleAcc = atan2(accX, accZ) * RAD_TO_DEG;
  // If the sign feels backwards, you can flip it:
  // angleAcc = atan2(-accX, accZ) * RAD_TO_DEG;

  // Integrate gyro (pitch rate stored in gyroX)
  gyroAngleX += gyroX * dt;

  // Complementary filter: blend gyro and accel
  currentAngle = alpha * (currentAngle + gyroX * dt) + (1.0f - alpha) * angleAcc;

  // Apply offset so upright ≈ 0°
  currentAngle -= angleOffset;
}


// ================== PID ==================
double calculatePID(double input) {
  unsigned long now = millis();
  double timeChange = (now - lastTime) / 1000.0;
  if (timeChange <= 0) timeChange = 0.01;

  double error = setpoint - input;

  double pTerm = Kp * error;

  integral += error * timeChange;
  integral = constrain(integral, -100, 100);
  double iTerm = Ki * integral;

  double dTerm = Kd * (error - lastError) / timeChange;

  double output = pTerm + iTerm + dTerm;

  lastError = error;
  lastTime  = now;

  return output;
}


// ================== MOTOR CONTROL (DRV8833) ==================
void controlMotors(double pidOutput) {
  int speed = (int)pidOutput;
  speed = constrain(speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);

  // Flip direction if needed (matches your previous working orientation)
  // If robot drives the wrong way when tilted, remove or add this negation
  speed = -speed;

  motorSpeedA = speed;
  motorSpeedB = speed;

  setMotorDRV8833(MOTOR_A_IN1, MOTOR_A_IN2, motorSpeedA);
  setMotorDRV8833(MOTOR_B_IN1, MOTOR_B_IN2, motorSpeedB);
}

/*
 * DRV8833 motor control:
 * - IN1=PWM, IN2=LOW  -> Forward at PWM duty cycle
 * - IN1=LOW, IN2=PWM  -> Reverse at PWM duty cycle
 * - IN1=LOW, IN2=LOW  -> Coast (motors free)
 * - IN1=HIGH, IN2=HIGH -> Brake (motors locked)
 */
void setMotorDRV8833(int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    // Forward: IN1=PWM, IN2=LOW
    analogWrite(in1Pin, speed);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    // Reverse: IN1=LOW, IN2=PWM
    digitalWrite(in1Pin, LOW);
    analogWrite(in2Pin, abs(speed));
  } else {
    // Coast: both LOW
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void stopMotors() {
  // Coast mode - motors free
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);

  motorSpeedA = 0;
  motorSpeedB = 0;
  integral = 0;
}

void brakeMotors() {
  // Brake mode - motors locked (short brake)
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


// ================== ULTRASONIC FUNCTIONS ==================
float measureDistanceCm() {
  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Echo with timeout (20 ms ~ 3.4 m)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);

  if (duration == 0) {
    return 0.0;   // no echo
  }

  // Speed of sound ~ 0.0343 cm/us, divide by 2 for round trip
  float dist = (duration * 0.0343f) / 2.0f;
  return dist;
}

void updateDriveCommand() {
  // Use filtered distance if available, otherwise raw distance
  float d = 0.0;

  if (filteredDistance > 0.0 && filteredDistance < MAX_VALID_DISTANCE) {
    d = filteredDistance;
  } else if (distanceCm > 0.0 && distanceCm < MAX_VALID_DISTANCE) {
    d = distanceCm;
  }

  // Default: stand still
  float desiredAngle = 0.0;

  // Only react if we have a valid reading within some range
  if (d > 0.0 && d < 150.0) {
    // --- BEHAVIOR MAP ---
    // Far:  70–150 cm -> drive forward (bigger tilt)
    // Mid:  40–70  cm -> slower forward
    // Near: 25–40  cm -> almost stop
    // Very close: <25 cm -> reverse
    if (d > FAR_DISTANCE) {
      desiredAngle = MAX_DRIVE_ANGLE;            // fast forward
    } else if (d > MID_DISTANCE) {
      desiredAngle = 0.5f * MAX_DRIVE_ANGLE;     // moderate forward
    } else if (d > NEAR_DISTANCE) {
      desiredAngle = 0.2f * MAX_DRIVE_ANGLE;     // slow forward / almost stop
    } else {
      desiredAngle = MAX_REVERSE_ANGLE;          // reverse
    }
  }

  // Save raw command (for debugging)
  driveAngleCmd = desiredAngle;

  // Slew-rate limit for smooth accel/decel & direction changes
  float delta = desiredAngle - driveAngleCmdSmooth;
  if (delta >  DRIVE_SLEW_PER_LOOP) delta =  DRIVE_SLEW_PER_LOOP;
  if (delta < -DRIVE_SLEW_PER_LOOP) delta = -DRIVE_SLEW_PER_LOOP;
  driveAngleCmdSmooth += delta;
}


// ================== POTENTIOMETER HELPERS (optional) ==================
int readPotentiometer() {
  return analogRead(POT_PIN);
}

float mapPotToFloat(int potValue, float minVal, float maxVal) {
  return minVal + (maxVal - minVal) * (potValue / 1023.0);
}
