#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::swingToHeading(double targetDeg, int timeoutMs,
                                        bool leftSidePivot, SwingToHeadingParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, targetDeg, timeoutMs, leftSidePivot, params]() { swingToHeading(targetDeg, timeoutMs, leftSidePivot, params, false);});
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }
    
    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;

    double lastTheta = pose.theta;

    swingPID.reset();
    swingSettle.reset();

    auto oldBrakeMode = motors.left->get_brake_mode();
    if (leftSidePivot) motors.right->set_brake_mode_all(toProsBrakeMode(BrakeMode::Hold));
    else motors.left->set_brake_mode_all(toProsBrakeMode(BrakeMode::Hold));

    auto [prevL, prevR] = motors.getLastVolt();
    double minV = fabs(params.minVolt), maxV = fabs(params.maxVolt);
    int dir = copysign(1, angleDiffDeg(targetDeg, pose.theta));
    
    MotionResult result = MotionResult::Timeout;
    int time = 0;
    const int stepMs = 10;
    const double dt = stepMs / 1000.0;
    std::uint32_t prevTime = pros::millis();

    bool motionChain = false;
    while (time < timeoutMs) {
        if (cancelRequested.load() || pros::competition::is_disabled()) {
            result = MotionResult::Cancelled;
            break;
        }
        
        if (!progressPaused) progress = progress + std::fabs(angleDiffDeg(pose.theta, lastTheta));
        lastTheta = pose.theta;

        double err = angleDiffDeg(targetDeg, pose.theta);

        if (chainExit(err, dir, minV, params.earlyExitRange)) {
            result = MotionResult::Chained;
            motionChain = true;
            break;
        }

        if (minV == 0 && swingSettle.update(err, dt)) {
            result = MotionResult::Settled;
            break;
        }

        double out = swingPID.calculateError(err, dt);
        if (std::fabs(err) > swingKsThreshold) out += std::copysign(swingKs, err);
        out = std::clamp(out, -maxV, maxV);
        if (minV > 0.0 && fabs(out) < minV) out = copysign(minV, err);
        double leftV = leftSidePivot ? limitOutput(-out, prevL, -maxV, maxV, params.slewRate) : 0.0 /* limitOutput(0.0, prevL, -maxV, maxV, params.slewRate) */;
        double rightV = leftSidePivot ? 0.0 /* limitOutput(0.0, prevR, -maxV, maxV, params.slewRate) */ : limitOutput(out, prevR, -maxV, maxV, params.slewRate);
    
        motors.setVoltage(leftV, rightV);
        if (leftSidePivot) motors.right->brake();
        else motors.left->brake();
        pros::Task::delay_until(&prevTime, stepMs);
        time += stepMs;
    }
    if (leftSidePivot) motors.right->set_brake_mode_all(oldBrakeMode);
    else motors.left->set_brake_mode_all(oldBrakeMode);

    if(!motionChain) motors.stop(BrakeMode::Brake);
    endMotion();
    return result;
}