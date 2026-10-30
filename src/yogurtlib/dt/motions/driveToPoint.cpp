#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::driveToPoint(double x, double y, int turnTimeout, int driveTimeout, 
                          DriveToPointParams params, bool async) {
    requestMotion();
    if (async){
        pros::Task task([this, x, y, turnTimeout, driveTimeout, params]() { driveToPoint(x, y, turnTimeout, driveTimeout, params, false);});
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }
    
    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;

    if (params.usePurePursuit) {
        std::vector<PathVec> points = {{pose.x, pose.y}, {x, y}};
        Path path(points, pathSettings);
        MotionResult result = curve(path, turnTimeout + driveTimeout, {.reversed = params.reversed, .minSpeed = (params.minVolt / 12000.0 * 127.0), .earlyExitRange = params.earlyExitRange,  .slewRate = params.slewRate} );
        endMotion();
        return result;
    }

    double targetHeading = pose.angleTo(Pose(x, y));
    int sign = 1;
    if (params.reversed) {
        targetHeading = wrapDeg(targetHeading + 180.0);
        sign = -1;
    }

    auto check = [](yogurt::MotionResult r) { //Lambda function to help us check motion state
        return (r == MotionResult::Chained) || (r == MotionResult::Settled);
    };
    progressPaused = true;
    auto turnR = turnToHeading(targetHeading, turnTimeout, {.maxVolt = params.maxVolt, .minVolt = driveToMinVolt, .earlyExitRange = driveToEarlyExit.first, .slewRate = params.slewRate}, false);
    
    if (turnR == MotionResult::Cancelled) { endMotion(); return turnR; }

    progressPaused = false;
    double dist = pose.distanceTo(Pose(x, y));
    double off = angleDiffDeg(targetHeading, pose.theta);
    auto driveR = driveStraight(sign * dist, driveTimeout, {.maxVolt = params.maxVolt, .minVolt = params.minVolt, .headingOffset = off, .earlyExitRange = params.earlyExitRange, .slewRate = params.slewRate}, false);

    endMotion();
    return !check(driveR)? driveR : (!check(turnR)? turnR : driveR); //If does not pass error check, return error otherwise normal output
}