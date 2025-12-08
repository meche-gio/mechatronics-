# MinSeg M1V4.3 Self-Balancing Robot - Arduino Mega 2560

A complete self-balancing robot implementation using PID control and MPU6050 IMU sensor for the MinSeg M1V4.3 shield on Arduino Mega 2560.

## Hardware Requirements

### MinSeg M1V4.3 Shield Components
- **Arduino Mega 2560** - Main microcontroller
- **MinSeg M1V4.3 Shield** - Plugs directly onto the Mega
  - **DRV8833** - Dual H-Bridge Motor Driver
  - **MPU6050** - 6-axis Gyroscope and Accelerometer (I2C)
  - **QMC5883** - 3-axis Compass (I2C, optional)
  - **Potentiometer** - For tuning
- **Lego NXT Motor** - DC motor with built-in encoder
- **9V Battery** or appropriate power supply

### Shield Features
- 2 motor/encoder headers
- Bluetooth communication header
- Ultrasonic sensor header
- Input potentiometer for tuning

## Pin Connections (M1V4.3 Shield)

The MinSeg M1V4.3 shield plugs directly onto the Arduino Mega 2560. The pins are pre-wired as follows:

### Motor Driver (DRV8833) - NOT L298N!

**Motor A:**
- IN1 (PWM) -> Pin 5
- IN2 (Direction) -> Pin 4

**Motor B:**
- IN1 (PWM) -> Pin 6
- IN2 (Direction) -> Pin 7

**Note:** The DRV8833 driver works differently than L298N:
- No separate enable pin
- Both IN pins control direction AND speed
- PWM on one pin + LOW on other = motor runs
- Both LOW = coast (gradual stop)
- Both HIGH = brake (quick stop)

### Encoder Connections (Lego NXT)

**Encoder A (Motor A):**
- Channel A -> Pin 2 (INT0)
- Channel B -> Pin 3 (INT1)

**Encoder B (Motor B):**
- Channel A -> Pin 18 (INT5)
- Channel B -> Pin 19 (INT4)

### MPU6050 IMU (I2C)

- SDA -> Pin 20 (fixed on Mega)
- SCL -> Pin 21 (fixed on Mega)
- I2C Address: 0x68

### Other Pins

- Potentiometer -> A0
- Status LED -> Pin 13

## Software Setup

### 1. Install Required Libraries

No external libraries needed! This code uses only built-in Arduino libraries:
- Wire.h (I2C communication)

### 2. Upload the Code

1. Open `m1v4_balance_code.ino` in Arduino IDE
2. Select **Board:** Tools -> Board -> Arduino Mega or Mega 2560
3. Select **Port:** Your Arduino's COM port
4. Click Upload

### 3. Verify Pin Configuration

Open `config.h` to see all pin assignments:
```cpp
// Motor A (DRV8833)
#define MOTOR_A_IN1   5    // PWM capable
#define MOTOR_A_IN2   4    // Direction

// Motor B (DRV8833)
#define MOTOR_B_IN1   6    // PWM capable
#define MOTOR_B_IN2   7    // Direction

// Encoders
#define ENCODER_A_PIN_A  2
#define ENCODER_A_PIN_B  3
// ... etc
```

## Initial Testing

### Step 1: Sensor Test
1. Upload the code
2. Open Serial Monitor (115200 baud)
3. Keep robot on a flat surface
4. Watch the angle readings - should be near 0 degrees when upright
5. Gyro calibration will run automatically on startup

### Step 2: Motor Direction Test
1. Hold the robot in the air
2. Tilt it forward - motors should spin to move forward
3. Tilt it backward - motors should reverse
4. **If motors spin wrong direction:** Swap IN1 and IN2 definitions in config.h

### Step 3: Encoder Test
1. Rotate motor shaft manually
2. Watch Serial Monitor for encoder count changes
3. Count should increase/decrease based on direction

### Step 4: Balance Test
1. Place robot upright on a smooth floor
2. Power on
3. Give it a gentle push - it should try to recover balance

## PID Tuning Guide

The robot uses PID control with three parameters:

```cpp
double Kp = 40.0;   // Proportional gain
double Ki = 0.8;    // Integral gain
double Kd = 1.2;    // Derivative gain
```

### Tuning Process

**Start with default values**, then adjust:

1. **Kp (Proportional)** - Main correction force
   - Too low: Robot falls over easily
   - Too high: Aggressive oscillations
   - Adjust by +/-5 at a time

2. **Kd (Derivative)** - Damping/smoothing
   - Too low: Overshoots and oscillates
   - Too high: Sluggish response
   - Adjust by +/-0.2 at a time

3. **Ki (Integral)** - Long-term drift correction
   - Too low: Slowly drifts forward/backward
   - Too high: Oscillations and instability
   - Start with 0 and increase slowly by +/-0.1

### Tuning Tips

- **Oscillating badly?** Reduce Kp and increase Kd
- **Falls over easily?** Increase Kp
- **Drifts forward/backward?** Slightly increase Ki
- **Very unstable?** Set Ki = 0 and tune Kp and Kd first

### Quick Presets (in config.h)

```cpp
// Aggressive: Kp=50, Ki=1.5, Kd=1.5
// Moderate: Kp=40, Ki=0.8, Kd=1.2 (default)
// Conservative: Kp=30, Ki=0.5, Kd=0.8
```

## DRV8833 Motor Control

The DRV8833 motor driver operates differently than L298N:

| IN1 | IN2 | Result |
|-----|-----|--------|
| PWM | LOW | Forward at PWM speed |
| LOW | PWM | Reverse at PWM speed |
| LOW | LOW | Coast (gradual stop) |
| HIGH | HIGH | Brake (quick stop) |

The code handles this automatically in the `setMotorDRV8833()` function.

## Troubleshooting

### Robot won't balance
- Check MPU6050 connections (SDA/SCL on pins 20/21)
- Verify motor directions are correct
- Ensure battery is fully charged
- Check that wheels can spin freely
- MPU6050 must be mounted level and secure on the shield

### Motors don't run
- Check shield is properly seated on Arduino Mega
- Verify power supply voltage
- Test motors by tilting robot while watching Serial Monitor
- Check PWM pins (5, 6) are connected correctly

### Robot oscillates wildly
- Reduce Kp gain
- Increase Kd gain
- Check for mechanical issues (loose parts)
- Ensure MPU6050 is firmly mounted on shield

### Encoder not counting
- Check encoder cable connection
- Verify interrupt pins (2, 3 or 18, 19) are correctly wired
- Test by rotating motor manually and watching Serial Monitor

### Drifts in one direction
- Check motor speeds are equal
- Verify wheels are same diameter
- Slightly adjust motor speeds in code
- Increase Ki very slightly

### Serial Monitor shows no data
- Check baud rate is 115200
- Verify USB connection
- Press reset button on Arduino
- Wait for gyro calibration to complete (~3 seconds)

## Code Structure

- `m1v4_balance_code.ino` - Main program with PID control and DRV8833 motor control
- `config.h` - Pin definitions and hardware configuration for M1V4.3 shield
- `README.md` - This file

## How It Works

1. **Sensor Reading** - MPU6050 measures tilt angle and rotation rate
2. **Gyro Calibration** - Automatically calibrates on startup (keep still!)
3. **Angle Calculation** - Complementary filter combines accelerometer and gyroscope
4. **PID Controller** - Calculates required motor power to maintain balance
5. **Motor Control** - DRV8833 driver applies power to motors to correct tilt
6. **Encoder Feedback** - Tracks wheel position for advanced control

### Complementary Filter
```
currentAngle = 0.96 x (gyro integration) + 0.04 x (accelerometer)
```
This blends fast gyroscope response with accurate accelerometer readings.

## Safety Notes

- Start testing on carpet or soft surface
- Hold robot when first testing
- Keep robot still during gyro calibration
- LED on Pin 13 indicates ready state
- Robot stops if tilt exceeds 45 degrees (safety cutoff)
- Keep fingers away from wheels when powered

## Resources

- [MinSeg.com](https://minseg.com) - Official MinSeg website
- [RASPLib](https://www.mathworks.com/matlabcentral/fileexchange/62702-rensselaer-arduino-support-package-library-rasplib) - Rensselaer Arduino Support Package Library
- [DRV8833 Datasheet](https://www.ti.com/product/DRV8833) - Motor driver documentation

## Version History

- **v4.3** - Updated for MinSeg M1V4.3 shield with DRV8833 motor driver
- Added encoder support
- Added gyro calibration with offset storage
- Added potentiometer support for tuning
- Tested on Arduino Mega 2560 with MinSeg M1V4.3 shield

---

**Happy Balancing!**
