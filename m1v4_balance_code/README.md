# M1V4.2 Self-Balancing Robot - State-Space Controller

Advanced self-balancing robot using state-space feedback control with MPU6050 IMU and wheel encoders for the Arduino Mega 2560 platform.

## Overview

This implementation uses a **state-space controller** (similar to LQR) that provides superior balance performance compared to simple PID control. The controller uses four state variables:

1. **Position** (x) - from wheel encoders
2. **Velocity** (ẋ) - from wheel encoders
3. **Tilt angle** (θ) - from IMU complementary filter
4. **Tilt rate** (θ̇) - from gyroscope

The control law is: `u = Kx·x + Kv·ẋ + Ktilt·θ + KtiltRate·θ̇`

## Hardware Requirements

### Essential Components
- **Arduino Mega 2560** - Main microcontroller
- **MPU6050** - 6-axis Gyroscope and Accelerometer
- **L298N Motor Driver** (or equivalent H-bridge driver)
- **2x DC Motors with Encoders** - 360 PPR (Pulses Per Revolution)
- **Battery** - 2S or 3S LiPo (7.4V or 11.1V)
- **Robot Chassis** - Two-wheeled design

### Recommended Specifications
- Motors: 6V-12V DC geared motors (200-300 RPM) with quadrature encoders
- Wheels: ~42mm diameter (wheelRadius = 0.021m)
- Encoders: 360 PPR (720 counts per revolution in quadrature)
- Battery: 2S LiPo (7.4V) with at least 1000mAh capacity
- Chassis: Stable two-wheeled platform with low center of gravity

## Pin Connections

### Motor Driver (L298N) to Arduino Mega

**Motor 1 (typically right):**
- IN1 (direction) → Pin 5
- IN2 (direction) → Pin 4
- EN (PWM speed) → Pin 11

**Motor 2 (typically left):**
- IN1 (direction) → Pin 9
- IN2 (direction) → Pin (controlled by motor2B logic)

**Power:**
- Motor Driver VCC → Battery +
- Motor Driver GND → Battery - and Arduino GND
- Arduino VIN → 7-12V (from battery or separate regulator)

### MPU6050 to Arduino Mega

- VCC → 5V
- GND → GND
- SDA → Pin 20 (SDA)
- SCL → Pin 21 (SCL)

**Important:** Most MPU6050 modules have built-in pull-up resistors. If not, add 4.7kΩ pull-ups on SDA and SCL.

### Encoders to Arduino Mega

**Encoder A (right wheel):**
- Channel A → Pin 2 (interrupt capable)
- Channel B → Pin 3 (interrupt capable)

**Note:** This code uses a single encoder for balance. For better drift correction, you can modify the code to use two encoders.

## Software Setup

### 1. Install Required Libraries

Install the **Encoder library** by Paul Stoffregen:
1. Open Arduino IDE
2. Go to: Sketch → Include Library → Manage Libraries
3. Search for "Encoder"
4. Install "Encoder by Paul Stoffregen"

Built-in libraries used:
- Wire.h (I2C communication)

### 2. Upload the Code

1. Open `m1v4_balance_code.ino` in Arduino IDE
2. Select **Board:** Tools → Board → Arduino Mega or Mega 2560
3. Select **Port:** Your Arduino's COM port
4. Click Upload

## Control Gain Tuning

The controller uses **optimized state-space feedback gains** (lines 25-28 in the code):

```cpp
float Kx        = 0.5f;       // position gain (m)
float Kv        = 2.5f;       // velocity gain (m/s)
float Ktilt     = 1500.0f;    // tilt angle gain (rad)
float KtiltRate = 200.0f;     // tilt rate gain (rad/s)
```

### What Each Gain Does

1. **Ktilt (Tilt Angle)** - Primary balance control
   - Higher = more aggressive correction when tilted
   - Too high = oscillations and instability
   - Too low = falls over easily
   - **Start here:** Adjust in steps of ±100

2. **KtiltRate (Tilt Rate / Damping)** - Prevents oscillation
   - Higher = more damping, smoother response
   - Too high = sluggish, delayed response
   - Too low = oscillates
   - **Adjust second:** Change in steps of ±20

3. **Kx (Position)** - Prevents drifting
   - Controls how strongly robot returns to position
   - Too high = jerky movements
   - Too low = drifts forward/backward
   - **Tune third:** Adjust in steps of ±0.1

4. **Kv (Velocity)** - Smooths position control
   - Adds damping to position control
   - Too high = sluggish position response
   - **Fine-tune last:** Adjust in steps of ±0.2

### Tuning Process

**Current gains are optimized** based on testing, but may need adjustment for your specific robot:

**If robot oscillates (shakes back and forth):**
1. Reduce `Ktilt` by 100 (e.g., 1500 → 1400)
2. Increase `KtiltRate` by 20 (e.g., 200 → 220)
3. Test and repeat until stable

**If robot falls easily:**
1. Increase `Ktilt` by 100 (e.g., 1500 → 1600)
2. May need to increase `KtiltRate` proportionally
3. Ensure battery is fully charged

**If robot drifts forward/backward:**
1. Increase `Kx` by 0.1 (e.g., 0.5 → 0.6)
2. May need to adjust `Kv` proportionally

**If robot is jerky or overshoots:**
1. Decrease `Kx` by 0.1
2. Increase `Kv` by 0.2

### Gain Optimization History

| Version | Kx  | Kv  | Ktilt  | KtiltRate | Notes                        |
|---------|-----|-----|--------|-----------|------------------------------|
| Initial | 0.6 | 2.9 | 1900   | 160       | Worked but oscillated        |
| **Tuned**   | **0.5** | **2.5** | **1500**   | **200**       | **Optimized for stability**  |

The tuned gains reduce oscillation by:
- Lowering tilt gain (1900 → 1500) for less aggressive correction
- Increasing damping (160 → 200) to absorb oscillations
- Reducing position/velocity gains slightly for smoother response

## Initial Testing

### Step 1: Verify Encoder Direction
1. Upload code and open Serial Monitor (115200 baud)
2. Manually roll the robot forward
3. Watch the position value increase
4. **If position decreases:** Uncomment line 271: `// pulses = -pulses;`

### Step 2: Verify Motor Direction
1. Hold robot in the air
2. Tilt it forward - wheels should spin to drive forward
3. Tilt it backward - wheels should spin backward
4. **If wrong:** Swap motor wire pairs at the motor driver

### Step 3: Calibrate Angle Offset
1. Place robot on flat surface in balanced position
2. Open Serial Monitor and observe tilt angle
3. Adjust `angleOffsetDeg` (line 39) so tilt reads near 0° when balanced
4. Example: If robot balances at -0.6°, set `angleOffsetDeg = 0.6f`

### Step 4: Balance Test
1. Place robot upright on smooth floor
2. Power on
3. Gently push - it should recover balance
4. If not stable, tune gains following section above

## Advanced Features

### Serial Communication Protocol

The robot sends telemetry data for visualization:
```
S<voltage>,<pos_error>,<velocity>,<abs_position>,<tilt>,<reference>
```

Example: `S2.1500,0.0120,0.0450,0.5320,-0.0050,0.5200`

You can send position commands via serial:
```
0.5\n      // Move to 0.5m forward
-0.3\n     // Move to 0.3m backward
0.0\n      // Return to origin
```

### Adjustable Parameters

**PWM Limits (lines 31-35):**
```cpp
int   maxPWM       = 220;    // Max power (max 255)
int   minPWM       = 40;     // Min to overcome friction
float deadzoneDeg  = 0.3f;   // Reduce power near balance
float safetyDeg    = 35.0f;  // Cutoff angle
```

**Sensor Filters (lines 118-120):**
```cpp
initFilter(accYFilter, 0.2f);      // Accelerometer Y smoothing
initFilter(accZFilter, 0.2f);      // Accelerometer Z smoothing
initFilter(velocityFilter, 0.3f);  // Velocity smoothing
```
Lower values = more filtering (slower response, less noise)

### Physical Adjustments

**Wheel Radius (line 68):**
```cpp
const float wheelRadius = 0.021f;  // 21mm in meters
```
Measure your wheel radius: diameter/2 in meters

**Encoder Resolution (line 274):**
```cpp
return pulses * 2.0f * pi / 720.0f;
```
Change 720.0f to match your encoder PPR × 2 (for quadrature)

## Troubleshooting

### Robot won't balance
- Verify MPU6050 connections (SDA/SCL on pins 20/21)
- Check encoder wiring (pins 2 and 3)
- Ensure battery is fully charged (>7V for 2S)
- Verify motors spin freely
- MPU6050 must be mounted level and rigidly

### Motors don't respond
- Check motor driver connections and power
- Verify PWM pins are connected
- Test motors with simple sketch
- Ensure battery can supply enough current

### Oscillates wildly
- **Reduce Ktilt** by 100-200
- **Increase KtiltRate** by 20-40
- Check for loose mechanical parts
- Ensure MPU6050 is firmly mounted
- Verify encoder signals are clean

### Drifts continuously
- **Increase Kx** (position gain)
- Check encoder is working (view serial output)
- Ensure both wheels same diameter
- Calibrate angle offset properly

### Falls over immediately
- **Increase Ktilt** by 100-200
- Check battery voltage (must be >7V)
- Verify motor direction is correct
- Ensure sufficient motor torque

### Serial shows garbage
- Verify baud rate is 115200
- Check USB cable connection
- Press reset button

## Performance Tips

1. **Smooth surface** - Hardwood or smooth concrete works best
2. **Battery placement** - Center between wheels, as low as possible
3. **Rigid construction** - No loose parts or flexible mounting
4. **Fresh battery** - Performance degrades below 7V
5. **Calibration** - Always calibrate on the surface you'll use
6. **Weight distribution** - Lower center of mass improves stability

## Comparison: State-Space vs PID

| Feature | Simple PID | State-Space (This Code) |
|---------|-----------|-------------------------|
| States controlled | 1 (angle only) | 4 (pos, vel, angle, rate) |
| Position control | No | Yes (encoder-based) |
| Drift correction | Limited | Excellent |
| Tuning difficulty | Moderate | Higher (more parameters) |
| Disturbance rejection | Good | Excellent |
| Stability | Good | Superior |

## Code Structure

- **m1v4_balance_code.ino** - Main program with state-space controller
- **config.h** - Pin definitions (legacy, not used by state-space version)
- **README.md** - This documentation

## How It Works

### Control Loop (125 Hz)
1. **Read Sensors** - MPU6050 (gyro + accel) and encoder
2. **State Estimation** - Complementary filter for angle, encoder for position/velocity
3. **Control Calculation** - State-space feedback: u = K×states
4. **Motor Command** - Convert control signal to PWM
5. **Serial Communication** - Send telemetry, receive commands

### Complementary Filter
```
angle = 0.96 × (gyro_integration) + 0.04 × (accelerometer)
```
- Fast gyro response (96%)
- Drift correction from accelerometer (4%)
- Prevents gyro drift while maintaining quick response

### State-Space Feedback
```
control = Kx×position + Kv×velocity + Ktilt×angle + KtiltRate×angular_rate
```
Each state contributes to the control decision, providing coordinated multi-objective control.

## Safety Notes

- Start testing on carpet or foam mat
- Hold robot during initial tests
- Keep fingers away from wheels
- Use proper LiPo charging safety
- Add physical bumpers to protect electronics
- Monitor battery voltage during operation
- Disconnect battery when not in use

## License

This code is open source and free for educational and personal projects.

## Version History

- **v4.2-StateSpace** (Current) - State-space controller with optimized gains
  - Ktilt reduced from 1900 to 1500 for stability
  - KtiltRate increased from 160 to 200 for damping
  - Position and velocity gains tuned for smooth response

- **v4.2** - Simple PID controller (legacy)
- **v4.0** - Initial Arduino Mega 2560 implementation

---

**Enjoy your self-balancing robot with advanced state-space control!**
