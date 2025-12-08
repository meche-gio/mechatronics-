/*
 * Configuration file for MinSeg M1V4.3 Balance Robot
 * Pin assignments and hardware configuration for Arduino Mega 2560
 *
 * This shield uses:
 * - DRV8833 Motor Driver (NOT L298N)
 * - MPU6050 IMU (I2C)
 * - QMC5883 Compass (I2C, optional)
 * - Lego NXT Motor with Encoder
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===== MOTOR DRIVER TYPE =====
// The M1V4.3 shield uses DRV8833, not L298N
// DRV8833 uses 2 pins per motor for direction AND PWM control
// (no separate enable pin like L298N)
#define USE_DRV8833

// ===== MOTOR A PINS (DRV8833) =====
// For DRV8833: IN1 and IN2 control both direction and speed
// PWM on one pin, LOW on other = forward
// LOW on first, PWM on other = reverse
// Both LOW = coast, Both HIGH = brake
#define MOTOR_A_IN1   5    // PWM capable pin (AIN1)
#define MOTOR_A_IN2   4    // Direction control (AIN2)

// ===== MOTOR B PINS (DRV8833) =====
// Second motor channel (if using dual motor setup)
#define MOTOR_B_IN1   6    // PWM capable pin (BIN1)
#define MOTOR_B_IN2   7    // Direction control (BIN2)

// ===== MPU6050 IMU PINS =====
// I2C communication (hardware I2C on Mega 2560)
// These are fixed hardware pins on the Mega
#define MPU_SDA      20    // SDA pin (fixed on Mega)
#define MPU_SCL      21    // SCL pin (fixed on Mega)
#define MPU6050_I2C_ADDR 0x68

// ===== ENCODER PINS =====
// Lego NXT encoder uses quadrature encoding
// Using interrupt-capable pins on Mega 2560
// Encoder 0 (Motor A): Pins 2, 3 (INT0, INT1)
#define ENCODER_A_PIN_A  2    // Channel A (interrupt pin)
#define ENCODER_A_PIN_B  3    // Channel B (interrupt pin)

// Encoder 1 (Motor B): Pins 18, 19 (INT5, INT4)
#define ENCODER_B_PIN_A  18   // Channel A (interrupt pin)
#define ENCODER_B_PIN_B  19   // Channel B (interrupt pin)

// Encoder specifications (Lego NXT motor)
#define ENCODER_CPR      360  // Counts per revolution
#define GEAR_RATIO       1.0  // Motor gear ratio

// ===== POTENTIOMETER PIN =====
// Input potentiometer on the shield
#define POT_PIN      A0   // Analog input for potentiometer

// ===== LED INDICATOR =====
#define LED_PIN      13   // Built-in LED for status

// ===== OPTIONAL: QMC5883 COMPASS =====
// Also on I2C bus, address 0x0D
#define QMC5883_I2C_ADDR 0x0D

// ===== ROBOT DIMENSIONS =====
// Lego NXT wheel specifications
#define WHEEL_DIAMETER    56.0   // mm (NXT wheel)
#define WHEEL_BASE        110.0  // mm (distance between wheels)
#define ROBOT_HEIGHT      100.0  // mm (approximate center of mass)

// ===== MOTOR SPECIFICATIONS =====
// Lego NXT Motor specs
#define MOTOR_MAX_RPM     170    // NXT motor max RPM
#define MOTOR_STALL_CURRENT 0.7  // Amps

// ===== PID TUNING PRESETS =====
// Uncomment one preset or use custom values in main code

// Aggressive (Fast response, may oscillate)
// #define PID_PRESET_AGGRESSIVE
// Kp = 50.0, Ki = 1.5, Kd = 1.5

// Moderate (Balanced) - Default
#define PID_PRESET_MODERATE
// Kp = 40.0, Ki = 0.8, Kd = 1.2

// Conservative (Slow but stable)
// #define PID_PRESET_CONSERVATIVE
// Kp = 30.0, Ki = 0.5, Kd = 0.8

// ===== SAFETY SETTINGS =====
#define FALL_ANGLE_THRESHOLD  45.0  // Degrees - stop motors if exceeded
#define MAX_PWM_VALUE         255   // Maximum PWM output

// ===== SERIAL DEBUG =====
#define SERIAL_BAUD_RATE  115200
#define DEBUG_INTERVAL    100     // ms between debug prints

#endif // CONFIG_H
