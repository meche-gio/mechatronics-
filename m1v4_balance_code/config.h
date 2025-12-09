/*
 * Configuration file for M1V4.3 Balance Robot
 * Pin assignments for Arduino Mega 2560 + DRV8833 Motor Driver
 */

#ifndef CONFIG_H
#define CONFIG_H

// ===== DRV8833 MOTOR DRIVER PINS =====
// DRV8833 uses 2 pins per motor (no separate enable pin)
// PWM on one pin, direction on the other

// Motor A (Right Motor)
#define MOTOR_A_IN1   9    // PWM capable (Timer 2)
#define MOTOR_A_IN2   10   // PWM capable (Timer 2)

// Motor B (Left Motor)
#define MOTOR_B_IN1   11   // PWM capable (Timer 1)
#define MOTOR_B_IN2   12   // Digital (direction)

// ===== HC-SR04 ULTRASONIC SENSOR (H3 HEADER) =====
#define TRIG_PIN      7    // D7 -> TRIG
#define ECHO_PIN      8    // D8 -> ECHO

// ===== MPU6050 IMU PINS =====
// I2C communication (hardware I2C on Mega 2560)
#define MPU_SDA       20   // SDA pin (fixed on Mega)
#define MPU_SCL       21   // SCL pin (fixed on Mega)

// ===== ENCODER PINS (Quadrature) =====
// Using interrupt-capable pins on Mega 2560
#define ENCODER_A_PIN_A  2   // Interrupt 0
#define ENCODER_A_PIN_B  4   // Digital
#define ENCODER_B_PIN_A  3   // Interrupt 1
#define ENCODER_B_PIN_B  5   // Digital

// ===== LED INDICATOR =====
#define LED_PIN       13   // Built-in LED

// ===== POTENTIOMETER (for tuning) =====
#define POT_PIN       A0   // Analog input for PID tuning

// ===== SERIAL CONFIGURATION =====
#define SERIAL_BAUD_RATE  115200

// ===== PWM CONFIGURATION =====
#define MAX_PWM_VALUE     255

// ===== ROBOT PHYSICAL PARAMETERS =====
#define WHEEL_DIAMETER    65.0   // mm
#define WHEEL_BASE        150.0  // mm (distance between wheels)
#define ROBOT_HEIGHT      120.0  // mm (center of mass height)

// ===== MOTOR SPECIFICATIONS =====
#define MOTOR_MAX_RPM     200    // Maximum motor RPM
#define GEAR_RATIO        1.0    // If using gearbox

// ===== FALL DETECTION =====
#define FALL_ANGLE_THRESHOLD  45.0  // degrees - robot considered fallen

// ===== PID TUNING PRESETS =====
// Starting values - tune for your specific robot

// For DRV8833 with small motors (N20/GA12):
// Start conservative and increase Kp until oscillation, then back off 20%
//
// Recommended tuning procedure:
// 1. Set Ki=0, Kd=0
// 2. Increase Kp until robot oscillates consistently
// 3. Reduce Kp by 20-30%
// 4. Increase Kd until oscillations dampen quickly
// 5. Add small Ki (0.5-2.0) to eliminate steady-state error

// Conservative starting values
#define DEFAULT_KP    55.0
#define DEFAULT_KI    0.8
#define DEFAULT_KD    1.8

// Aggressive (after tuning)
// #define DEFAULT_KP    80.0
// #define DEFAULT_KI    1.5
// #define DEFAULT_KD    2.5

#endif // CONFIG_H
