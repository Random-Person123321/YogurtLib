#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

MotionResult Drivetrain::boomerang(double x, double y, double heading, 
                        double timeOutMs, BoomParams params, bool async) {
    requestMotion();
    if (async) {
        pros::Task task([this, x, y, heading, timeOutMs, params]() { boomerang(x, y, heading, timeOutMs, params, false);});
        endMotion();
        pros::delay(10);
        return MotionResult::Running;   
    }

    if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
    if (params.slewRate < 0.0) params.slewRate = defaultSlew;
    
    drivePID.reset();
    turnPID.reset();
    boomerangDriveSettle.reset();
    boomerangTurnSettle.reset();

    auto [prevL, prevR] = motors.getLastVolt();
    double prevDrive = (prevL + prevR) / 2.0;

    params.lead = std::clamp(params.lead, 0.0, 1.0);
    double minV = fabs(params.minVolt), maxV = fabs(params.maxVolt);
    int dir = params.reversed ? -1 : 1;
    
    bool close = false;
    int stillMs = 0;
    double lastX = pose.x, lastY = pose.y, lastTheta = pose.theta;
    
    MotionResult result = MotionResult::Timeout;
    int elapsed = 0;
    const int stepMs = 10;
    const double dt = stepMs / 1000.0;
    std::uint32_t prevTime = pros::millis();

    bool motionChain = false;

    while (elapsed < timeOutMs) { 
        if (cancelRequested.load() || pros::competition::is_disabled()){
            result = MotionResult::Cancelled;
            break;
        }
        Pose p = pose;
        double stepDist = std::hypot(p.x - lastX, p.y - lastY);
        double stepTurn = std::fabs(angleDiffDeg(p.theta, lastTheta));
        //Update progress:
        if (!progressPaused) progress = progress + stepDist;
        
        lastX = p.x;
        lastY = p.y;
        lastTheta = p.theta;

        double dx = x - p.x, dy = y - p.y;
        double d = std::hypot(dx, dy);
        if (!close && d < params.closeDist) {
            close = true;
            maxV = std::min(maxV, std::max(std::fabs(prevDrive), boomerangCloseMinVolt));
        }

        //Math for carrot point 
        double travelDeg = heading + (params.reversed ? 180.0 : 0.0);         
        double travelRad = deg2rad(travelDeg); 
        double carrotX = close ? x : x - d * params.lead * std::cos(travelRad);
        double carrotY = close ? y : y - d * params.lead * std::sin(travelRad);

        double robotDeg = p.theta + (params.reversed ? 180.0 : 0.0);
        double robotRad = deg2rad(robotDeg);

        double carrotDx = carrotX - p.x, carrotDy = carrotY - p.y;
        double wantDeg = close ? travelDeg : rad2deg(std::atan2(carrotDy, carrotDx));
        double headErr = angleDiffDeg(wantDeg, robotDeg);
        
        int sign = std::cos(deg2rad(headErr)) > 0 ? 1 : -1;
        double carrotDist = std::hypot(carrotDx, carrotDy);
        double fwd = close ? dx * std::cos(robotRad) + dy * std::sin(robotRad) : carrotDist * sign;
        double distErr = dir * fwd; 

        bool driveDone = boomerangDriveSettle.update(distErr, dt);
        bool turnDone = boomerangTurnSettle.update(headErr, dt);            

        //Math for motionchaining
        double along = dx * std::cos(travelRad) + dy * std::sin(travelRad);
        
        if (close && minV > 0.0 && along < params.earlyExitRange) {
            result = MotionResult::Chained;
            motionChain = true;
            break;
        }

        if (minV == 0.0 && close && driveDone && turnDone) {
            result = MotionResult::Settled;
            break;
        }
        if (close && stepDist < 0.01 && stepTurn < 0.05) stillMs += stepMs;
        else stillMs = 0;
        if (minV == 0.0 && stillMs >= 250) {
            result = MotionResult::Stuck;
            break;
        }
        double driveOut = drivePID.calculateError(distErr, dt);
        double turnOut = turnPID.calculateError(headErr, dt);

        
        if(std::fabs(distErr) > lateralKsThreshold) driveOut += std::copysign(lateralKs, distErr);
        if(close && std::fabs(headErr) > turnKsThreshold) turnOut += std::copysign(turnKs, headErr);
        if (!close && params.curveK > 0.0) {
            double s = std::fabs(std::sin(deg2rad(headErr)));
            if (s >= 1e-3) {
                double radius = carrotDist / (2.0 * s);
                double limit = params.curveK * std::sqrt(radius);
                driveOut = std::clamp(driveOut, -limit, limit);
            }
        }
        driveOut = std::clamp(driveOut, -maxV, maxV);
        turnOut = std::clamp(turnOut, -maxV, maxV);
        double over = std::fabs(driveOut) + std::fabs(turnOut) - maxV;
        if (over > 0.0) driveOut -= std::copysign(over, driveOut);
        if (minV > 0.0 && fabs(driveOut) < minV) driveOut = std::copysign(minV, distErr);
        if (!close) {
            if (dir > 0) driveOut = std::max(driveOut, 0.0);
            else if (dir < 0) driveOut = std::min(driveOut, 0.0);
            driveOut = slew(driveOut, prevDrive, params.slewRate);
        }

        double leftV = driveOut - turnOut;
        double rightV = driveOut + turnOut;
        double m = std::max(fabs(leftV), fabs(rightV));
        if (m > maxV) {
            leftV *= maxV / m;
            rightV *= maxV / m;
        }
            
        prevDrive = driveOut;

        motors.setVoltage(leftV, rightV);
        pros::Task::delay_until(&prevTime, stepMs);
        elapsed += stepMs;
    }

    if(!motionChain) motors.stop(BrakeMode::Brake);
    endMotion();
    return result;
}