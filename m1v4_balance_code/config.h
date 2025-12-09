/*
 * Configuration file for M1V4.3 Balance Robot
 * Pin assignments and hardware configuration for Arduino Mega 2560
 * Hardware: MinSeg Shield with DRV8833 motor driver, MPU6050 IMU, HC-SR04 sonar
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===== SERIAL CONFIGURATION =====
#define SERIAL_BAUD_RATE  115200
#define DEBUG_INTERVAL    100     // ms between debug prints

// ===== MOTOR A PINS (Right Motor) - DRV8833 Style =====
// MinSeg Shield M1 connector
#define MOTOR_A_IN1   4    // M1A - PWM/Direction pin 1
#define MOTOR_A_IN2   5    // M1B - PWM/Direction pin 2

// ===== MOTOR B PINS (Left Motor) - DRV8833 Style =====
// MinSeg Shield M2 connector (adjust if different on your board)
#define MOTOR_B_IN1   6    // M2A - PWM/Direction pin 1
#define MOTOR_B_IN2   9    // M2B - PWM/Direction pin 2

// ===== PWM CONFIGURATION =====
#define MAX_PWM_VALUE 255  // Maximum PWM value (0-255)

// ===== MPU6050 IMU PINS =====
// I2C communication (hardware I2C on Mega 2560)
#define MPU_SDA      20    // SDA pin (fixed on Mega)
#define MPU_SCL      21    // SCL pin (fixed on Mega)

// ===== ENCODER PINS =====
// Motor A encoder (using interrupt-capable pins)
#define ENCODER_A_PIN_A  2    // E1 - Interrupt pin (INT4 on Mega)
#define ENCODER_A_PIN_B  3    // E2 - Interrupt pin (INT5 on Mega)

// Motor B encoder (if using second encoder)
#define ENCODER_B_PIN_A  18   // Interrupt pin (INT3 on Mega)
#define ENCODER_B_PIN_B  19   // Interrupt pin (INT2 on Mega)

// ===== HC-SR04 ULTRASONIC PINS =====
// MinSeg Shield H3 sonar header
#define SONAR_TRIG_PIN   7    // TRG
#define SONAR_ECHO_PIN   8    // ECH

// ===== LED INDICATOR =====
#define LED_PIN      13    // Built-in LED

// ===== POTENTIOMETER (Optional tuning) =====
#define POT_PIN      A0    // Analog pin for potentiometer

// ===== FALL DETECTION =====
#define FALL_ANGLE_THRESHOLD  45.0  // degrees - robot considered fallen

// ===== ROBOT DIMENSIONS =====
#define WHEEL_DIAMETER    65.0   // mm
#define WHEEL_BASE        150.0  // mm (distance between wheels)
#define ROBOT_HEIGHT      120.0  // mm (center of mass height)

// ===== MOTOR SPECIFICATIONS =====
#define MOTOR_MAX_RPM     200    // Maximum motor RPM
#define GEAR_RATIO        1.0    // If using gearbox

// ===== PID TUNING PRESETS =====
// Uncomment one preset or use custom values in main code

// Aggressive (Fast response, may oscillate)
// #define PID_PRESET_AGGRESSIVE
// Kp = 180.0, Ki = 0.3, Kd = 7.5

// Moderate (Balanced)
#define PID_PRESET_MODERATE
// Kp = 120.0, Ki = 0.5, Kd = 5.0

// Conservative (Slow but stable)
// #define PID_PRESET_CONSERVATIVE
// Kp = 80.0, Ki = 0.3, Kd = 3.0

#endif // CONFIG_H
