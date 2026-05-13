#ifndef DRIVE_H_
#define DRIVE_H_
#include "main.h"
int driveStraight(double distanceInches, 
                  int timeoutMs = 5000,
                  double headingOffset = 0.0,
                  double maxVoltage = 11000.0, 
                  double slewRate = 900.0,
                  double minVoltage = 0.0);

void turnToAngle(double targetAngle,
                 int timeoutMs = 3000,
                 double maxVoltage = 11000.0,
                 double slewRate = 900.0);

FollowResult followPath(const std::vector<std::pair<double, double>>& path,
                double lookaheadDist = 14.0,   // inches — start here, tune 10-18
                double maxVoltage = 11000.0,   // mV
                int timeoutMs = 15000,
                double slewRate = 700.0,       // mV per 10ms
                PursuitDir dir = PursuitDir::Forward,
                bool progressCheck = true);    // For

FollowResult driveToPoint(double targetX, double targetY, double maxVoltage = 11000.0, bool usePurePursuit = true, bool backwards = false, int turnTimeout = 900, int driveTimeout = 1000);

FollowResult driveToPose(double targetX, double targetY, double targetHeading, double maxVoltage = 11000.0, bool usePurePursuit = true, bool backwards = false, int turnTimeout = 900, int driveTimeout = 1000);

void turnToPoint(double targetX, double targetY, int timeout = 1000, double maxVoltage = 11000.0, bool back = false);

void swingToHeading(double targetHeading, bool leftSidePivot = true, int timeoutMs = 3000, double maxVoltage = 8000.0, double slewRate = 700.0);
#endif
