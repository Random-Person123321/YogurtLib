#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

    // Drives to (x, y) then turns to face `headingDeg`.
MotionResult Drivetrain::driveToPose(double x, double y, double headingDeg,
                            int turnTimeout, int driveTimeout,
                            DriveToPoseParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, x, y, headingDeg, turnTimeout, driveTimeout, params]() { driveToPose(x, y, headingDeg, turnTimeout, driveTimeout, params, false);});
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }

    auto check = [](yogurt::MotionResult r) { //Lambda function to help us check motion state
        return (r == MotionResult::Chained) || (r == MotionResult::Settled);
    };

    MotionResult pointR = driveToPoint(x, y, turnTimeout, driveTimeout,  {.maxVolt = params.maxVolt, .minVolt = driveToMinVolt, .usePurePursuit = params.usePurePursuit, .reversed = params.reversed, .earlyExitRange = driveToEarlyExit.second, .slewRate = params.slewRate}, false);
    if (pointR == MotionResult::Cancelled) { endMotion(); return pointR; }

    progressPaused = true;
    auto turnR = turnToHeading(headingDeg, turnTimeout, {.maxVolt = params.maxVolt, .minVolt = params.minVolt, .earlyExitRange = params.earlyExitRange, .slewRate = params.slewRate}, false);
    
    endMotion();
    return !check(pointR)? pointR : (!check(turnR)? turnR : pointR);
}