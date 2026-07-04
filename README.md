# Robotrick

A clean, high-level Arduino movement library for a differential-drive robot on an **Arduino Mega 2560** (built for the *Robotrick Mega Shield* / WRO Junior 2026). It turns messy motor/sensor code into one-line commands:

```cpp
bot.begin();
bot.forward(30);        // drive straight 30 cm (encoder distance + gyro heading-hold)
bot.turnLeft(90);       // accurate gyro turn
bot.followLineToJunction(1);   // follow the line, stop at the black cross-line
```

## Features

- **Planned-trajectory straight driving** — trapezoid speed profile (accelerate → cruise → decelerate) + feedforward + tracking PID for smooth, accurate `forward()` / `backward()`.
- **Gyro turns** — magnitude-based two-phase turn (L3GD20H), immune to wheel slip.
- **Line follower** — QTR reflectance array, peak-windowed squared-weight centroid, PD control, junction detection, line-loss recovery. **Calibration saved to EEPROM** (calibrate once, remembered forever).
- **Selectable straightness** — gyro, encoder-difference, or both (`RT_STRAIGHT_MODE`).
- **4th motor** control (lift / mechanism) by speed + time.
- **One central config block** in `src/Robotrick.h` — every pin, geometry value, speed, and gain in one place. Tune without touching logic.

## Hardware

- Arduino Mega 2560
- 2× drive motors (M1 left, M2 right) with quadrature encoders on interrupt pins
- L3GD20H gyro (I2C, 0x6B)
- QTR reflectance sensor array (RC type)
- Optional 4th motor (lift/mechanism)

## Install

**Option A — git clone** into your Arduino `libraries` folder:
```bash
cd <your-sketchbook>/libraries
git clone git@github.com:Waelalhamad/Robotrick_Library.git Robotrick
```

**Option B — ZIP:** Download this repo as ZIP → Arduino IDE → *Sketch → Include Library → Add .ZIP Library*.

Then restart the Arduino IDE.

### Dependencies (install via Library Manager)
- **Encoder** by Paul Stoffregen
- **QTRSensors** by Pololu

## Quick start

```cpp
#include <Robotrick.h>
Robotrick bot;

void setup() {
    bot.begin();            // init motors, encoders, gyro + calibrate gyro
    bot.forward(30);
    bot.turnLeft(90);
    bot.forward(50);
    bot.stop();
}
void loop() {}
```

## API

| Category | Methods |
|----------|---------|
| Setup | `begin()`, `calibrateGyro()`, `calibrateSpeed()` |
| Move | `forward(cm)`, `backward(cm)`, `turnLeft(deg)`, `turnRight(deg)`, `stop()` |
| Line | `lineCalibrate()`, `lineLoadCalibration()`, `followLineToJunction(n)`, `followLineForCM(cm)` |
| Motor 4 | `motor4(speed)`, `motor4For(speed, ms)`, `motor4Stop()` |
| Sensors | `heading()`, `resetHeading()`, `encoderLeft()`, `encoderRight()`, `linePosition()` |
| Low-level | `setMotors(l, r)` |

## Configuration

All tuning lives in **`src/Robotrick.h`** between the `CONFIG` markers: motor pins, wheel geometry & distance calibration, drive/turn/line speeds, PID gains, tolerances, and safety timeouts. Nothing else needs editing.

## Examples

- **RobotControl** — USB serial bridge (drive + live telemetry) for a desktop app.
- **TestBench** — serial-menu tester for every function.
- **MapSolve** — a mission written as a recipe.
- **TuneTurn**, **QTRMiddle** — calibration/tuning helpers.

## License

MIT — see [LICENSE](LICENSE).
