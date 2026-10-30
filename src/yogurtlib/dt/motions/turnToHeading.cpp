#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::turnToHeading(double targetDeg, int timeoutMs, TurnToHeadParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, targetDeg, timeoutMs, params]() { turnToHeading(targetDeg, timeoutMs, params, false); });
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }

    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;

    turnPID.reset();
    turnSettle.reset();
    double lastTheta = pose.theta;
    MotionResult result = MotionResult::Timeout;

    auto [prevL, prevR] = motors.getLastVolt();
    double minV = fabs(params.minVolt), maxV = fabs(params.maxVolt);
    int dir = copysign(1, angleDiffDeg(targetDeg, pose.theta));

    int elapsed = 0;
    const int stepMs = 10;
    const double dt = stepMs / 1000.0;
    std::uint32_t prevTime = pros::millis();

    bool motionChain = false;

    while (elapsed < timeoutMs) {
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
        if (minV == 0.0 && turnSettle.update(err, dt)) {
                result = MotionResult::Settled;
                break;
        }

        double out = turnPID.calculateError(err, dt);
        if (std::fabs(err) > turnKsThreshold) out += std::copysign(turnKs, err);
        out = std::clamp(out, -maxV, maxV);
        if (minV > 0.0 && fabs(out) < minV) out = copysign(minV, err);
        double leftV = limitOutput(-out, prevL, -maxV, maxV, params.slewRate);
        double rightV = limitOutput(out, prevR, -maxV, maxV, params.slewRate);
        motors.setVoltage(leftV, rightV);
        
        pros::Task::delay_until(&prevTime, stepMs);
        elapsed += stepMs;
    }
    if(!motionChain) motors.stop(BrakeMode::Brake);
    endMotion();
    return result;
}