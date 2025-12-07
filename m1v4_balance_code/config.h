/*
 * Configuration file for M1V4.2 Balance Robot
 * Pin assignments and hardware configuration for Arduino Mega 2560
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===== MOTOR A PINS (Right Motor) =====
// Using L298N Motor Driver
#define MOTOR_A_IN1   2    // Direction pin 1
#define MOTOR_A_IN2   3    // Direction pin 2
#define MOTOR_A_EN    4    // PWM Enable pin (must be PWM capable)

// ===== MOTOR B PINS (Left Motor) =====
#define MOTOR_B_IN1   5    // Direction pin 1
#define MOTOR_B_IN2   6    // Direction pin 2
#define MOTOR_B_EN    7    // PWM Enable pin (must be PWM capable)

// ===== MPU6050 IMU PINS =====
// I2C communication (hardware I2C on Mega 2560)
#define MPU_SDA      20    // SDA pin (fixed on Mega)
#define MPU_SCL      21    // SCL pin (fixed on Mega)

// ===== OPTIONAL ENCODER PINS =====
// Uncomment if using encoders for position/speed feedback
// #define ENCODER_A_PIN_A  18  // Interrupt pin
// #define ENCODER_A_PIN_B  19  // Interrupt pin
// #define ENCODER_B_PIN_A  20  // Interrupt pin
// #define ENCODER_B_PIN_B  21  // Interrupt pin

// ===== LED INDICATOR (Optional) =====
#define LED_PIN      13    // Built-in LED

// ===== BATTERY MONITORING (Optional) =====
// #define BATTERY_PIN  A0    // Analog pin for battery voltage
// #define BATTERY_MIN  6.6   // Minimum voltage (for 2S LiPo)
// #define BATTERY_MAX  8.4   // Maximum voltage (for 2S LiPo)

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
// Kp = 50.0, Ki = 1.5, Kd = 1.5

// Moderate (Balanced)
#define PID_PRESET_MODERATE
// Kp = 40.0, Ki = 0.8, Kd = 1.2

// Conservative (Slow but stable)
// #define PID_PRESET_CONSERVATIVE
// Kp = 30.0, Ki = 0.5, Kd = 0.8

#endif // CONFIG_H
