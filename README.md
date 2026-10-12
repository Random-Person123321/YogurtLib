<p align="center">
  <img width="300" alt="YogurtLib logo" src="https://github.com/user-attachments/assets/26ccebf1-18d0-40a7-ae7c-94e7968af4be" />
</p>

# YogurtLib

**A C++ autonomous library for VEX V5 robots on PROS: tracking-wheel and multi-IMU odometry, PID motions you can chain together, and a touchscreen UI with an auton selector for the brain.**

<!-- HERO: replace with a GIF/video of your robot running an auton, ideally with the brain screen visible. -->
<p align="center">
  <img src="docs/demo.gif" width="600" alt="Robot running a YogurtLib autonomous routine" />
</p>

<p align="center">
  <a href="https://github.com/Random-Person123321/YogurtLib/releases/tag/YogurtLib_UI_%26_OOP"><b>⬇️ Download the newest release</b></a>
  &nbsp;·&nbsp;
  <a href="https://yogurtlib.com/home"><b>🧭 Try the web tools (path planner, data visualizer)</b></a>
</p>

---

## Quick start

You'll need the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/index.html) and an existing PROS 4 project. Download `yogurtlib@1.0.0.zip` from the [latest release](https://github.com/Random-Person123321/YogurtLib/releases/latest), then run these from your project folder:

```bash
pros c fetch yogurtlib@1.0.0.zip
pros c apply yogurtlib
```

Include it in `src/main.cpp`, describe your robot, and drive:

```cpp
#include "main.h"
#include "yogurtlib/api.hpp"

// Drive motors (negative port = reversed)
pros::MotorGroup leftMotors({-19, 20});
pros::MotorGroup rightMotors({-18, 17});
yogurt::DriveMotors motors(&leftMotors, &rightMotors, 2.75);   // wheel diameter (in)

// Odometry: a vertical and a horizontal tracking wheel, plus 1-3 IMUs
pros::Rotation vertical(4), horizontal(5);
pros::Imu imu1(1), imu2(2), imu3(3);
std::vector<pros::Imu*> imus = {&imu1, &imu2, &imu3};

yogurt::OdomSensors odom(&vertical, 2.0, 0.25,      // vertical wheel: diameter, offset (in)
                         &horizontal, 2.0, -2.7,    // horizontal wheel: diameter, offset (in)
                         imus,
                         nullptr, nullptr, nullptr, nullptr,  // distance sensors (optional)
                         0.0, 0.0, 0.0, 0.0);

//                                  kP     kI   kD    maxI  errTol velTol settleMs
yogurt::ControllerSettings lateral(600.0, 0.0, 0.0,  3.0,  0.1,   0.5,   1000);
yogurt::ControllerSettings turn   (120.0, 0.0, 15.0, 2.0,  0.75,  8.0,   200);
yogurt::ControllerSettings swing  (200.0, 0.0, 16.0, 3.0,  0.75,  8.0,   200);

yogurt::Drivetrain chassis(motors, odom, lateral, turn, swing);

void initialize() {
    chassis.calibrate(yogurt::Pose(0, 0, 90));   // resets IMUs, starts odometry
}

void autonomous() {
    chassis.driveStraight(24, 2000);              // 24 in forward
    chassis.turnToHeading(0, 1000);               // face east
    chassis.driveToPoint(48, 24, 1000, 2000, {.usePurePursuit = false});
}
```

The example gains are from our own robot. You'll need to tune them for yours.

## Features

- **Odometry from tracking wheels and up to three IMUs.** Pose runs in its own task at 10 ms. Headings from all IMUs are averaged on the unit circle, and a disconnected IMU is skipped, so one bad sensor doesn't throw off the whole run.
- **Six motion types:** `driveStraight`, `turnToHeading`, `turnToPoint`, `swingToHeading`, `driveToPoint`, and `driveToPose`. Each returns whether it settled, timed out, or was cancelled.
- **Async motions and motion chaining.** Pass `async = true` to keep running other code (intakes, lifts) while the robot drives. Call `waitUntil(n)` to trigger something once the robot has covered `n` inches (or turned `n` degrees) of a motion. Set a `minVolt` above zero and the robot carries its speed into the next motion instead of stopping.
- **Slew rate and static-friction feedforward on every motion.** Slew prevents wheel slip when the robot speeds up, and the kS push stops it stalling short of the target.
- **Automatic tracking-wheel offset tuning.** `tuneOffset()` spins the robot both ways and measures where your tracking wheels actually sit, so you don't have to measure them with a ruler.
- **Brain-screen UI.** Touchscreen pages show live X, Y, heading and each IMU's reading, plus an auton selector and a button to recalibrate the IMUs.
- **Web tools at [yogurtlib.com](https://yogurtlib.com/home).** A Bézier path planner that exports paths in the library's format, a data visualizer for replaying pose logs, and an LVGL image converter for custom brain-screen images.

### In progress
- **Adaptive pure pursuit** (based on the Dawgma path-following paper): separate path generation, curvature-based slowdown, and velocity feedforward. Until it lands, use `driveToPoint` with `usePurePursuit = false`.
- **Monte Carlo Localization** using the four distance sensors and a beam sensor model to correct odometry drift against the field walls. It's being prototyped in a browser simulator first, then ported to the brain.

## Building from source

Requirements: the PROS CLI with kernel **4.2.2** and the ARM toolchain that comes with it, plus a V5 brain for testing.

```bash
git clone https://github.com/Random-Person123321/YogurtLib.git
cd YogurtLib
pros make template      # builds yogurtlib@1.0.0.zip, the same file shipped in releases
```

`src/main.cpp` is excluded from the template, so you can use it as a test program. Run `pros mu` to upload it to a brain.

## Conventions

- **Angles follow the unit circle:** 0° = east, 90° = north, counterclockwise is positive. Autons assume the robot starts at 90°, facing away from the alliance wall.
- **Distances are in inches. Motion output is in millivolts**, capped at `maxVoltage` (12000 by default).
- **Slew is measured in mV per 10 ms loop.** The default is 600.

## How it works

**Voltage control instead of the motors' built-in velocity PID.** V5 motors have their own velocity controller (`move_velocity`), but it responds slowly and behaves differently depending on the load. YogurtLib sets voltage directly and closes every loop itself, using odometry. That means *full speed* is the fastest the robot can actually go under load, not a number in rpm on a spec sheet.

**Trusting the IMUs over the distance sensors for heading.** Distance sensors can be thrown off by other robots and game elements, while three fused IMUs stay reliable for a full match. So heading always comes from the IMUs. Once MCL arrives, the particle filter will only estimate X and Y and won't touch heading. That gives it one fewer variable to estimate, which matters on a Cortex-A9 with a 10 ms loop.

**One `Drivetrain` object instead of global functions.** Hardware, PID controllers, the pose estimate, and the odometry task all live in one class. Settings that have sensible defaults are fields you can change after construction, rather than constructor arguments you have to fill in. Optional parts like the UI and MCL hook in through `onCalibrated` callbacks, so teams that don't need them don't pay for them.

## Credits

- Built by Eric (team **210E**).
- [PROS](https://pros.cs.purdue.edu/), the kernel and toolchain YogurtLib runs on.
- [LVGL](https://lvgl.io/), which powers the brain-screen UI.
- [LemLib](https://github.com/LemLib/LemLib), a big inspiration for the API style (0–127 speeds, motion chaining).
- Team 1712 Dawgma's *Implementation of the Adaptive Pure Pursuit Controller*, the basis for the path follower.
