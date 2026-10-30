#pragma once
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace yogurt {
  
// *** Angle Helpers ***
double wrapDeg(double a);

// returns shortest signed difference target - current in degrees
double angleDiffDeg(double targetDeg, double currentDeg);

double deg2rad(double d);

double rad2deg(double r);

double wrapRad(double a);

// Convert PROS get_heading() [0,360] to signed [-180,180]
double headingToSignedDeg(double h);

} // namspace yogurt