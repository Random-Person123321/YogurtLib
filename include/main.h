/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"
// #include "lv_global.h"
#define M_PI 3.14159265358979323846

//Constants for Tuning (Per robot, dont assume)
const double VERT_DIAMETER = 2.75;
const double HORIZ_DIAMETER = 0.0; //Set to 0 if not using
const double TRACK_WIDTH = 10.5; //Measure
const double HORIZ_FWD_OFFSET = 0.0; //Forward = positive, set to 0 if not using
const double VERT_RIGHT_OFFSET = 0.38; //Right = positive

// const double LOOKAHEAD = 12.0;

// Define Hardware with extern here if using multiple files
struct Drivetrain{
    pros::Motor& lfront;
    pros::Motor& lmiddle;
    pros::Motor& lback;
    pros::Motor& rfront;
    pros::Motor& rmiddle;
    pros::Motor& rback;
};

enum struct PursuitDir {
  Forward,
  Reverse, 
  Auto
};

enum struct FollowResult {
  ReachedEnd,
  Timeout,
  NoIntersection_TargetBehind,
  Stuck,
  PathTooShort
};

// ---- Angle helpers (degrees) ----
inline double wrapDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// returns shortest signed difference target - current in degrees
inline double angleDiffDeg(double targetDeg, double currentDeg) {
  return wrapDeg(targetDeg - currentDeg);
}

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

inline double wrapRad(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// Convert PROS get_heading() [0,360) to signed (-180,180]
inline double headingToSignedDeg(double h) {
  if (h > 180.0) h -= 360.0;
  return wrapDeg(h);
}

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

inline double batteryScale() {
  // Scale outputs so behavior stays similar as battery sags
  int mv = pros::battery::get_voltage();      // typically up to ~12000
  if (mv < 9000) mv = 9000;                  // avoid extreme scaling
  return 12000.0 / (double)mv;
}

struct RobotPose { double x = 0.0, y = 0.0, heading = 0.0; };
extern RobotPose pose;
extern double odomOffset;

extern pros::Controller master;

extern pros::Rotation verticalOdom;
extern pros::Rotation horizontalOdom;
extern pros::Rotation cata;

extern pros::Imu imu1;
extern pros::Imu imu2;
extern std::vector <pros::Imu> imus;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor intake;
extern pros::Motor catapult;
extern pros::MotorGroup intakes;

extern pros::adi::DigitalOut Matchload;
extern pros::adi::DigitalOut Blocker;
extern pros::adi::DigitalOut Mid;
extern pros::adi::DigitalOut Bunny;
extern pros::adi::DigitalOut Lift;

extern pros::Distance bD;
extern pros::Distance LD;
extern pros::Distance fD;
extern pros::Distance rD;

extern bool activateJamDetection;
extern bool activateIntakeStop;
extern bool home;
extern double startPosition;
const extern double HcataRange;
const extern double LcataRange;
extern bool Parked;
extern int startTime;
extern int timeUsed;
extern bool UseTime;
// ---- PID (dt aware, competition-grade) ----
struct PID {
  double kP;
  double kI;
  double kD;

  double maxIntegral;   // clamp on integral term magnitude (0 disables clamp)
  double integral = 0.0;
  double prevError = 0.0;
  bool first = true;

  PID(double p, double i, double d, double maxInt)
      : kP(p), kI(i), kD(d), maxIntegral(maxInt) {}

  void reset() {
    integral = 0.0;
    prevError = 0.0;
    first = true;
  }

  // Best method: feed error directly
  double calculateError(double error, double dt) {
    if (dt <= 0) dt = 0.01;

    integral += error * dt;
    if (maxIntegral != 0) {
      integral = std::clamp(integral, -maxIntegral, maxIntegral);
    }

    double derivative = 0.0;
    if (!first) derivative = (error - prevError) / dt;
    first = false;

    prevError = error;
    return kP * error + kI * integral + kD * derivative;
  }

  // Standard method: target/current
  double calculate(double target, double current, double dt) {
    return calculateError(target - current, dt);
  }

  // Backwards compatible (assumes 10ms loop)
  double calculate(double target, double current) {
    return calculate(target, current, 0.01);
  }
};


/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include "init.h"
#include "sensors.h"
#include "drive.h"
#include "odometry.h"
#include "misc.h"
#include "pre_auton.h"
#include "autonomous.h"
#include "paths.h"
#include "main.h"
#include "brain_photo.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
