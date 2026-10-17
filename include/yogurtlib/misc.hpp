#pragma once
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// *** Angle Helpers ***
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