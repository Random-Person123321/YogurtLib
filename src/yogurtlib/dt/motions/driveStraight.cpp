#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::driveStraight(double distanceInches, int timeoutMs, DriveParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, distanceInches, timeoutMs, params]() {driveStraight(distanceInches, timeoutMs, params, false); });
        endMotion();
        pros::delay(10);
        return MotionResult::Running;
    }
    
    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;

    drivePID.reset();
    angularPID.reset();
    driveSettle.reset();

    double startHeading = wrapDeg(pose.theta + params.headingOffset);
    double targetX = pose.x + distanceInches * std::cos(deg2rad(startHeading));
    double targetY = pose.y + distanceInches * std::sin(deg2rad(startHeading));
    double lastX = pose.x, lastY = pose.y;
    
    auto [prevL, prevR] = motors.getLastVolt();
    double minV = fabs(params.minVolt), maxV = fabs(params.maxVolt);
    int dir = copysign(1, distanceInches);

    MotionResult result = MotionResult::Timeout;
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

        if (!progressPaused) progress = progress + std::hypot(pose.x - lastX, pose.y - lastY);
        lastX = pose.x;
        lastY = pose.y;

        double dx = targetX - pose.x;
        double dy = targetY - pose.y;
        double cosH = std::cos(deg2rad(startHeading));
        double sinH = std::sin(deg2rad(startHeading));
        double distErr = dx * cosH + dy * sinH;
        double headErr = angleDiffDeg(startHeading, pose.theta);
        
        if (chainExit(distErr, dir, minV, params.earlyExitRange)) {
            result = MotionResult::Chained;
            motionChain = true;
            break;
        }
        
        if (minV == 0.0 && driveSettle.update(distErr, dt)) {
            result = MotionResult::Settled;
            break;
        }
        double driveOut = drivePID.calculateError(distErr, dt);
        double turnOut = angularPID.calculateError(headErr, dt);
        
        if (std::fabs(distErr) > lateralKsThreshold) driveOut += std::copysign(lateralKs, distErr);
        driveOut = std::clamp(driveOut, -maxV, maxV);
        if (minV > 0.0 && fabs(driveOut) < minV) driveOut = copysign(minV, distErr);
        
        //Desaturation
        double leftV = driveOut - turnOut;
        double rightV = driveOut + turnOut;
        double m = std::max(fabs(leftV), fabs(rightV));
        if (m > maxV) { //Scale both down by same factor so larger side is maxVoltage while the other side is appropriately scaled down to respect turnOutput
            leftV *= maxV / m;
            rightV *= maxV / m;
        }
        leftV = limitOutput(leftV, prevL, -maxV, maxV, params.slewRate);
        rightV = limitOutput(rightV, prevR, -maxV, maxV, params.slewRate);

        motors.setVoltage(leftV, rightV);
        pros::Task::delay_until(&prevTime, stepMs);
        elapsed += stepMs;
    }
    if(!motionChain) motors.stop(BrakeMode::Brake);
    endMotion();
    return result;

}