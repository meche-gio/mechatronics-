# M1V4.2 Self-Balancing Robot - Arduino Mega 2560

A complete self-balancing robot implementation using PID control and MPU6050 IMU sensor for the Arduino Mega 2560 platform.

## Hardware Requirements

### Essential Components
- **Arduino Mega 2560** - Main microcontroller
- **MPU6050** - 6-axis Gyroscope and Accelerometer
- **L298N Motor Driver** (or equivalent H-bridge driver)
- **2x DC Motors** - Geared motors with wheels
- **Battery** - 2S or 3S LiPo (7.4V or 11.1V)
- **Robot Chassis** - Two-wheeled design

### Recommended Specifications
- Motors: 6V-12V DC geared motors (200-300 RPM)
- Wheels: 65mm diameter
- Battery: 2S LiPo (7.4V) with at least 1000mAh capacity
- Chassis: Stable two-wheeled platform

## Pin Connections

### Motor Driver (L298N) to Arduino Mega

**Motor A (Right):**
- IN1 → Pin 2
- IN2 → Pin 3
- EN (PWM) → Pin 4

**Motor B (Left):**
- IN1 → Pin 5
- IN2 → Pin 6
- EN (PWM) → Pin 7

**Power:**
- Motor Driver VCC → Battery +
- Motor Driver GND → Battery - and Arduino GND
- Arduino VIN → 7-12V (from battery or separate power)

### MPU6050 to Arduino Mega

- VCC → 5V
- GND → GND
- SDA → Pin 20 (SDA)
- SCL → Pin 21 (SCL)

**Important:** Use 4.7kΩ pull-up resistors on SDA and SCL if not built-in on your MPU6050 module.

## Software Setup

### 1. Install Required Libraries

No external libraries needed! This code uses only built-in Arduino libraries:
- Wire.h (I2C communication)

### 2. Upload the Code

1. Open `m1v4_balance_code.ino` in Arduino IDE
2. Select **Board:** Tools → Board → Arduino Mega or Mega 2560
3. Select **Port:** Your Arduino's COM port
4. Click Upload

### 3. Verify Pin Configuration

Open `config.h` and verify the pin assignments match your wiring:
```cpp
#define MOTOR_A_IN1   2
#define MOTOR_A_IN2   3
#define MOTOR_A_EN    4
// ... etc
```

## Initial Testing

### Step 1: Sensor Test
1. Upload the code
2. Open Serial Monitor (115200 baud)
3. Keep robot on a flat surface
4. Watch the angle readings - should be near 0° when upright

### Step 2: Motor Direction Test
1. Hold the robot in the air
2. Tilt it forward - motors should spin to move forward
3. Tilt it backward - motors should reverse
4. **If motors spin wrong direction:** Swap IN1 and IN2 wires for that motor

### Step 3: Balance Test
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
   - Adjust by ±5 at a time

2. **Kd (Derivative)** - Damping/smoothing
   - Too low: Overshoots and oscillates
   - Too high: Sluggish response
   - Adjust by ±0.2 at a time

3. **Ki (Integral)** - Long-term drift correction
   - Too low: Slowly drifts forward/backward
   - Too high: Oscillations and instability
   - Start with 0 and increase slowly by ±0.1

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

## Troubleshooting

### Robot won't balance
- Check MPU6050 connections (SDA/SCL)
- Verify motor directions are correct
- Ensure battery is fully charged
- Check that wheels can spin freely
- MPU6050 must be mounted level and secure

### Motors don't run
- Check motor driver connections
- Verify power supply voltage (7-12V)
- Test motors directly with battery
- Check PWM pins are connected correctly

### Robot oscillates wildly
- Reduce Kp gain
- Increase Kd gain
- Check for mechanical issues (loose parts)
- Ensure MPU6050 is firmly mounted

### Drifts in one direction
- Check motor speeds are equal
- Verify wheels are same diameter
- Slightly adjust motor speeds in code
- Increase Ki very slightly

### Serial Monitor shows no data
- Check baud rate is 115200
- Verify USB connection
- Press reset button on Arduino

## Advanced Features

### Adding Remote Control
You can add Bluetooth or IR remote control by:
1. Reading remote commands in loop()
2. Adjusting `setpoint` variable to lean forward/backward
3. Adding separate turn control by varying motor speeds

### Speed Control Example
```cpp
int forwardCommand = 0; // -100 to +100 from remote
setpoint = forwardCommand * 0.1; // Lean to move
```

### Battery Monitoring
Uncomment battery monitoring in config.h and add voltage divider:
```cpp
#define BATTERY_PIN A0
float batteryVoltage = analogRead(BATTERY_PIN) * (5.0/1023.0) * VOLTAGE_DIVIDER_RATIO;
```

## Performance Tips

1. **Use smooth floor** - Carpet adds resistance
2. **Balance battery weight** - Center battery between wheels
3. **Tight mechanical build** - No loose parts
4. **Fresh batteries** - Low voltage = poor performance
5. **Calibrate on flat surface** - Very important!

## Code Structure

- `m1v4_balance_code.ino` - Main program with PID control
- `config.h` - Pin definitions and configuration
- `README.md` - This file

## How It Works

1. **Sensor Reading** - MPU6050 measures tilt angle and rotation rate
2. **Angle Calculation** - Complementary filter combines accelerometer and gyroscope
3. **PID Controller** - Calculates required motor power to maintain balance
4. **Motor Control** - Applies power to motors to correct tilt

### Complementary Filter
```
currentAngle = 0.96 × (gyro integration) + 0.04 × (accelerometer)
```
This blends fast gyroscope response with accurate accelerometer readings.

## Safety Notes

- Start testing on carpet or soft surface
- Hold robot when first testing
- Use proper battery handling (LiPo safe charging)
- Add physical bumpers to protect components
- Keep fingers away from wheels when powered

## License

This code is open source and free to use for educational and personal projects.

## Support

For issues or questions about this code:
1. Check troubleshooting section above
2. Verify all connections
3. Review Serial Monitor output for debugging

## Version History

- **v4.2** - Current release with optimized PID and complementary filter
- Tested on Arduino Mega 2560
- Compatible with MPU6050 IMU

---

**Happy Balancing!**
