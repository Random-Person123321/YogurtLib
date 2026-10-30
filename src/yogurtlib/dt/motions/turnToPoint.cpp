#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, x, y, timeoutMs, params]() { turnToPoint(x, y, timeoutMs, params, false); });
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }

    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;
    
    double targetHeading = pose.angleTo(Pose(x, y));
    if (params.reversed) targetHeading = wrapDeg(targetHeading + 180.0);
    MotionResult result = turnToHeading(targetHeading, timeoutMs, {.maxVolt = params.maxVolt, .minVolt = params.minVolt, .earlyExitRange = params.earlyExitRange, .slewRate = params.slewRate}, false);
    endMotion();
    return result;
}