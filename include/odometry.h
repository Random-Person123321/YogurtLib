#ifndef ODOMETRY_H_
#define ODOMETRY_H_

// namespace pros { class Distance; }
#include "api.h"

double getCurrentHeading(); // returns heading in degrees, wrapped to (-180, 180]

void resetOdometry(double startX = 0.0, double startY = 0.0, double startHeading = 0.0);

void updateOdometry(void* param);

// Distance-based relocalization (blended)
void distanceReset(pros::Distance& sensor,
                   double knownFeaturePos,
                   double sensorToCenterOffset,
                   bool isX,
                   bool sensorDirPositive,
                   double angleOffset = 0.0);

void terminalDebug();

#endif
