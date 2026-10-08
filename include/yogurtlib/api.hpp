#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
  PathTooShort,
  Cancelled, 
  Running
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

// Convert PROS get_heading() [0,360] to signed [-180,180]
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

extern int startTime;


struct particle_data {
	double x, y, h, weight;
};

extern std::vector<particle_data> particles;

#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/mcl.hpp"
#include "yogurtlib/pose.hpp"
#include "yogurtlib/dt/motors.hpp"
#include "yogurtlib/UI.hpp"
#include "yogurtlib/global.hpp"