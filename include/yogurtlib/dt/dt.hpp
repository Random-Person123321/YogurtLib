#pragma once
#include "main.h"
#include <atomic>
#include <functional>

namespace yogurt {

// Bundles a PID's gains with the Settle config used to decide when a motion
// driven by that PID has finished.
struct ControllerSettings {
    double kp, ki, kd, maxIntegral;
    double errTol, velTol;
    int settleMs;

    ControllerSettings(double p, double i, double d, double maxInt,
                        double errTol, double velTol, int settleMs)
        : kp(p), ki(i), kd(d), maxIntegral(maxInt),
          errTol(errTol), velTol(velTol), settleMs(settleMs) {} 
    
};

// Odometry hardware: tracking wheels, IMUs, and wall-distance sensors. 
// Leave a pointer null to disable that sensor. Aggregate type —
// construct with designated initializers, e.g.
//   OdomSensors odom{.vertical = &vert, .verticalOffset = 0.38, .imus = {&imu1, &imu2}};
// The distance sensors exist purely so a future MCL localizer has hardware
// access; Drivetrain itself doesn't read them.
struct OdomSensors {
    pros::Rotation* vertical = nullptr;
    double verticalDiameter = 2.75;   // inches
    double verticalOffset = 0.0;      // inches, right-of-center is positive

    pros::Rotation* horizontal = nullptr;
    double horizontalDiameter = 2.75; // inches
    double horizontalOffset = 0.0;    // inches, forward-of-center is positive

    std::vector<pros::Imu*> imus = {};

    pros::Distance* leftD = nullptr;
    pros::Distance* frontD = nullptr;
    pros::Distance* rightD = nullptr;
    pros::Distance* backD = nullptr;

    double leftOffset = 0.0;
    double rightOffset = 0.0;
    double frontOffset = 0.0;
    double backOffset = 0.0;

    OdomSensors(pros::Rotation* vertical, double verticalDiameter, double verticalOffset,
                pros::Rotation* horizontal, double horizontalDiameter, double horizontalOffset,
                std::vector<pros::Imu*> imus, pros::Distance* leftD, pros::Distance* frontD,
                pros::Distance* rightD, pros::Distance* backD, double leftOffset, double rightOffset,
                double frontOffset, double backOffset) :
                vertical(vertical), verticalDiameter(verticalDiameter), verticalOffset(verticalOffset),
                horizontal(horizontal), horizontalDiameter(horizontalDiameter), horizontalOffset(horizontalOffset),
                imus(imus), leftD(leftD), frontD(frontD), rightD(rightD), backD(backD),
                leftOffset(leftOffset), rightOffset(rightOffset), frontOffset(frontOffset), backOffset(backOffset) {}

};

// Tunable pure-pursuit profile. Defaults are reasonable starting points —
// retune per-robot on the Drivetrain instance, not by editing this header.
struct PursuitProfile {
    double stopDist = 2.0;        // inches to the end point at which to stop
    double endSlowDist = 24.0;    // start slowing down within this of the end
    double minBase = 2200.0;      // minimum base voltage
    double curvClampK = 0.98;     // clamp on normalized curvature (track/2 / R), avoids wheel reversal
    double curvSlowK = 1.1;       // how hard to slow down for curvature
    double endSlowMinFrac = 0.35; // minimum fraction of max voltage allowed near the end
    double forwardTol = 0.5;      // inches; lookahead points must be at least this far in front
    int stuckMs = 500;            // if no path progress for this long -> Stuck
    int noLookFailMs = 800;       // if no lookahead intersection for this long -> try recovery

    //No need for a default constructor to allow aggregates
};

struct DriveParams {
    double headingOffset = 0.0;
    double maxVolt = -1.0;
    double minVolt = 0.0;
    double earlyExitRange = 0.0;
    double slewRate = -1.0; 
};

struct TurnToHeadParams {
    double maxVolt = -1.0;
    double slewRate = -1.0; 
};



struct TurnToPointParams {
    double maxVolt = -1.0;
    bool reversed = false;
};

struct SwingToHeadingParams {
    double maxVolt = -1.0;
    double slewRate = -1.0;
};

struct DriveToPointParams {
    bool usePurePursuit = true; 
    bool reversed = false;
    double maxVolt = -1.0;
};

struct DriveToPoseParams {
    double maxVolt = -1.0;
    bool usePurePursuit = true;
    bool reversed = false;
};

// Outcome of a blocking point/turn motion (driveStraight, turnToHeading,
// turnToPoint, swingToHeading).
enum class MotionResult {
    Settled,
    Timeout,
    Cancelled,
    Running
};

inline std::string entostr (FollowResult result) {
    int num = (int)result;
    // switch (num){
    //     case 0: return "ReachEnd";
    //     case 1: return "Timeout";
    // }
    if (num == 0) {
        return "ReachedEnd";
    }
    else if (num == 1) {
        return "Timeout";
    }
    else if (num == 2) {
        return "NoIntersection_TargetBehind";
    }
    else if (num == 3) {
        return "Stuck";
    }
    else if (num == 4) {
        return "PathTooShort";
    }
    else return "Unknown";
}



// Top-level drivetrain object: owns the drive hardware, PID controllers,
// pose estimate, and every drive/turn/path-following motion. This is the
// object autonomous code should talk to; it replaces the free functions in
// drive.cpp/odometry.cpp with one cohesive class.
class Drivetrain {
public:
    DriveMotors motors;
    OdomSensors odom;
    PursuitProfile pursuit;
    
    //Default maxVoltage and slew
    double maxVoltage = 12000.0;
    double defaultSlew = 800.0;

    // Static-friction feedforward: a small extra push (mV) added on top of
    // the PID output so the robot actually starts moving instead of stalling
    // against its own static friction right as a motion begins or ends. One per
    // motion type — tune per-robot
    double lateralKs = 900.0;
    double turnKs = 1100.0;
    double swingKs = 900.0;

    // How large the error has to be (inches/degrees) before the friction push above kicks in. 
    //Below this, no push is added to avoids a jolt right as a motion is settling in.
    double lateralKsThreshold = 1.0;
    double turnKsThreshold = 1.5;
    double swingKsThreshold = 2.0;

    // Only the pieces with no sane default are constructor arguments: the
    // physical hardware, and the three gain sets a Drivetrain can't do its
    // job without (drive/turn/swing). Everything else above is a settable
    // field with a working default, or configured via a setter below
    // (see setHeadingCorrectionGains) — call those after construction if you
    // want non-default behavior, instead of one long argument list.
    Drivetrain(DriveMotors motors, OdomSensors odomSensors,
               ControllerSettings driveSettings, ControllerSettings turnSettings, //Drive for driveStraight, turn for turnning
               ControllerSettings swingSettings) : //swing for sing settings
               motors(motors), odom(odomSensors),
               drivePID(driveSettings.kp, driveSettings.ki, driveSettings.kd, driveSettings.maxIntegral), //DriveStraight
               turnPID(turnSettings.kp, turnSettings.ki, turnSettings.kd, turnSettings.maxIntegral), // Turning
               swingPID(swingSettings.kp, swingSettings.ki, swingSettings.kd, swingSettings.maxIntegral), //Swing logic
               driveSettle{driveSettings.errTol, driveSettings.velTol, driveSettings.settleMs}, // Drivestraight
               turnSettle{turnSettings.errTol, turnSettings.velTol, turnSettings.settleMs}, //Settle for turning from turnSettings
               swingSettle{swingSettings.errTol, swingSettings.velTol, swingSettings.settleMs} {}

    // PID for heading correction during driveStraight to maintain a straight line(or intentional angling)
    // Call it once during setup if you want driveStraight to hold heading.
    void setHeadingCorrectionGains(double kp, double ki, double kd, double maxIntegral) {
        angularPID = PID(kp, ki, kd, maxIntegral);
    }

    // ---- Setup ----
    std::vector<std::function<void(Drivetrain&)>> onCalibrated;

    // Blocking IMU reset + starts the odometry task. Call once from initialize().
    void calibrate(Pose startPose = Pose(0, 0, 0)) {
        for (auto* imu : odom.imus) {
            imu->reset(false);
            pros::delay(10);
            imu->set_data_rate(5);
        }
        pros::delay(2000);
        setPose(startPose);
        if (!odomTaskStarted) {
            odomTaskStarted = true;
            // The task keeps running independently of this handle's lifetime.
            pros::Task odomTask([this] { this->odomLoop(); });
        }
        for (auto& cb : onCalibrated) cb(*this); //Runs on the functions and passes the this pointer as part of the class in
    }

    // Requests that any in-progress blocking motion stop at its next
    // iteration. Safe to call from another task (e.g. a disable handler).
    void cancelMotion() { cancelRequested = true; }
    
    //Warning, this will physically stop the motors but will not kill the functions running the motors
    //To cancel the running of the functions call cancelMotion()
    //Errorenous use of this function could cause your program to freeze
    void stop() { motors.stop(toBrakeMode(motors.left->get_brake_mode())); }

    // ---- Pose ----

    Pose getPose() const { return pose; }

    void setPose(const Pose& p) {
        pose = p;

        double compassHeading = wrapDeg(90.0 - pose.theta);
        if (compassHeading < 0) compassHeading += 360.0;
        for (auto* imu : odom.imus) imu->set_heading(compassHeading);

        if (odom.vertical) lastVertTicks = odom.vertical->get_position();
        if (odom.horizontal) lastHorizTicks = odom.horizontal->get_position();
        lastHeadingDeg = pose.theta;
    }

    // ---- Motions ----

    // Drives a length in inches along the current (or offset) heading, holding
    // heading with the angular PID. Positive distance drives forward.
    MotionResult driveStraight(double distanceInches, int timeoutMs, DriveParams params = {}, bool async = false) {
        requestMotion();
        if (async) {
            pros::Task task([=, this]() { driveStraight(distanceInches, timeoutMs, params, false); });
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
        int dir = copysign(1, distanceInches);
        double minV = fabs(params.minVolt), maxV = fabs(params.maxVolt);

        double targetX = pose.x + distanceInches * std::cos(deg2rad(startHeading));
        double targetY = pose.y + distanceInches * std::sin(deg2rad(startHeading));

        double prevL = 0.0, prevR = 0.0;
        double lastX = pose.x, lastY = pose.y;

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
            
            if (minV > 0 && (distErr * dir < 0 || fabs(distErr) < params.earlyExitRange)) {
                result = MotionResult::Settled;
                motionChain = true;
                break;
            }
            
            if (minV == 0 && driveSettle.update(distErr, dt)) {
                result = MotionResult::Settled;
                break;
            }

            double driveOut = drivePID.calculateError(distErr, dt);
            double turnOut = angularPID.calculateError(headErr, dt);
            
            if (std::fabs(distErr) > lateralKsThreshold) driveOut += std::copysign(lateralKs, distErr);
            driveOut = clampd(driveOut, -maxV, maxV);
            if (minV > 0 && fabs(driveOut) < minV) driveOut = copysign(minV, distErr);
            
            //Desaturation
            double leftV = driveOut - turnOut;
            double rightV = driveOut + turnOut;
            int m = std::max(fabs(leftV), fabs(rightV));
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

    // Turns in place to face `targetDeg`.
    MotionResult turnToHeading(double targetDeg, int timeoutMs, TurnToHeadParams params = {}, bool async = false) {
        requestMotion();
        if (async) {
            pros::Task task([=, this]() { turnToHeading(targetDeg, timeoutMs, params, false); });
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
        double prevV = 0.0;
        int elapsed = 0;
        const int stepMs = 10;
        const double dt = stepMs / 1000.0;
        std::uint32_t prevTime = pros::millis();

        while (elapsed < timeoutMs) {
            if (cancelRequested.load() || pros::competition::is_disabled()) {
                result = MotionResult::Cancelled;
                break;
            }
            if (!progressPaused) progress = progress + std::fabs(angleDiffDeg(pose.theta, lastTheta));
            lastTheta = pose.theta;

            double err = angleDiffDeg(targetDeg, pose.theta);
            if (std::fabs(err) < 0.5) err = 0.0;
            if (turnSettle.update(err, dt)) {
                result = MotionResult::Settled;
                break;
            }

            double out = turnPID.calculateError(err, dt);
            if (std::fabs(err) > turnKsThreshold) out += std::copysign(turnKs, err);

            double v = limitOutput(out, prevV, -params.maxVolt, params.maxVolt, params.slewRate);

            motors.setVoltage(-v, v);
            pros::Task::delay_until(&prevTime, stepMs);
            elapsed += stepMs;
        }

        motors.stop(BrakeMode::Brake);
        endMotion();
        return result;
    }

    // Turns in place to face the point (x, y).
    MotionResult turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params = {}, bool async = false) {
        requestMotion();
        if (async) {
            pros::Task task([=, this]() { turnToPoint(x, y, timeoutMs, params, false); });
            endMotion();
            pros::delay(10);
            return MotionResult::Running;
        }
        
        double targetHeading = pose.angleTo(Pose(x, y));
        if (params.reversed) targetHeading = wrapDeg(targetHeading + 180.0);
        MotionResult result = turnToHeading(targetHeading, timeoutMs, {.maxVolt = params.maxVolt});
        endMotion();
        return result;
    }

    // Pivots about one side only (the other side stays stopped) to face `targetDeg`.
    MotionResult swingToHeading(double targetDeg, int timeoutMs,
                                bool leftSidePivot = true, SwingToHeadingParams params = {}, bool async = false) {
        requestMotion();
        if (async) {
            pros::Task task([=, this]() { swingToHeading(targetDeg, timeoutMs, leftSidePivot, params, false);});
            endMotion();
            pros::delay(10);
            return MotionResult::Running;
        }
        
        if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;
        if (params.slewRate < 0.0) params.slewRate = defaultSlew;

        double lastTheta = pose.theta;

        swingPID.reset();
        swingSettle.reset();

        MotionResult result = MotionResult::Timeout;
        int time = 0;
        const int stepMs = 10;
        const double dt = stepMs / 1000.0;
        double prevV = 0.0;
        std::uint32_t prevTime = pros::millis();

        while (time < timeoutMs) {
            if (cancelRequested.load() || pros::competition::is_disabled()) {
                result = MotionResult::Cancelled;
                break;
            }
            
            if (!progressPaused) progress = progress + std::fabs(angleDiffDeg(pose.theta, lastTheta));
            lastTheta = pose.theta;

            double err = angleDiffDeg(targetDeg, pose.theta);
            if (swingSettle.update(err, dt)) {
                result = MotionResult::Settled;
                break;
            }

            double out = swingPID.calculateError(err, dt);
            if (std::fabs(err) > swingKsThreshold) out += std::copysign(swingKs, err);

            double v = limitOutput(out, prevV, -params.maxVolt, params.maxVolt, params.slewRate);

            if (leftSidePivot) motors.setVoltage(-v, 0.0);
            else motors.setVoltage(0.0, v);

            pros::Task::delay_until(&prevTime, stepMs);
            time += stepMs;
        }

        motors.stop(BrakeMode::Brake);
        endMotion();
        return result;
    }

    // Drives to (x, y): pure pursuit by default, or turn-then-drive if
    // usePurePursuit is false.
    FollowResult driveToPoint(double x, double y, int turnTimeout, int driveTimeout, 
                              DriveToPointParams params = {}, bool async = false) {
        requestMotion();
        if (async){
            pros::Task task([=, this]() { driveToPoint(x, y, turnTimeout, driveTimeout, params, false);});
            endMotion();
            pros::delay(10);
            return FollowResult::Running;
        }
        
        if (params.maxVolt < 0.0) params.maxVolt = maxVoltage;

        if (params.usePurePursuit) {
            std::vector<std::pair<double, double>> path = {{pose.x, pose.y}, {x, y}};
            PursuitDir dir = params.reversed ? PursuitDir::Reverse : PursuitDir::Forward;
            FollowResult result = follow(path, 11.5, params.maxVolt, driveTimeout, 700.0, dir, false);
            endMotion();
            return result;
        }

        double targetHeading = pose.angleTo(Pose(x, y));
        int sign = 1;
        if (params.reversed) {
            targetHeading = wrapDeg(targetHeading + 180.0);
            sign = -1;
        }
        progressPaused = true;
        if (turnToHeading(targetHeading, turnTimeout, {.maxVolt = params.maxVolt}) == MotionResult::Cancelled) {
            endMotion();
            return FollowResult::Cancelled;
        }
        progressPaused = false;
        double dist = pose.distanceTo(Pose(x, y));
        if (driveStraight(sign * dist, driveTimeout, {.maxVolt = params.maxVolt}) == MotionResult::Cancelled) {
            endMotion();
            return FollowResult::Cancelled;
        }
        endMotion();
        return FollowResult::ReachedEnd;
    }

    // Drives to (x, y) then turns to face `headingDeg`.
    FollowResult driveToPose(double x, double y, double headingDeg,
                             int turnTimeout, int driveTimeout,
                             DriveToPoseParams params = {}, bool async = false) {
        requestMotion();
        if (async) {
            pros::Task task([=, this]() { driveToPose(x, y, headingDeg, turnTimeout, driveTimeout, params, false);});
            endMotion();
            pros::delay(10);
            return FollowResult::Running;
        }
        FollowResult result = driveToPoint(x, y, turnTimeout, driveTimeout, {.usePurePursuit = params.usePurePursuit, .reversed = params.reversed, .maxVolt = params.maxVolt});
        if (result == FollowResult::Cancelled) {
            endMotion();
            return result;
        }
        phase = 1;
        progressPaused = true;
        if (turnToHeading(headingDeg, turnTimeout, {.maxVolt = params.maxVolt}) == MotionResult::Cancelled) {
            endMotion();
            return FollowResult::Cancelled;
        }
        endMotion();
        return result;
    }

    // Pure pursuit path following over a polyline of (x, y) waypoints.
    FollowResult follow(const std::vector<std::pair<double, double>>& path,
                         double lookaheadDist = 14.0, double maxVolt = -1.0,
                         int timeoutMs = 15000, double slewRate = 700.0,
                         PursuitDir dir = PursuitDir::Forward, bool progressCheck = true) {
        if (maxVolt < 0.0) maxVolt = maxVoltage;
        if (path.size() < 2) return FollowResult::PathTooShort;

        cancelRequested = false;

        const int startTime = pros::millis();
        size_t lastSeg = 0;
        size_t bestProgressSeg = 0;
        double prevL = 0.0, prevR = 0.0;
        const int stepMs = 10;
        std::uint32_t prevTime = pros::millis();

        const double Ld = std::max(lookaheadDist, 1.0);

        FollowResult result = FollowResult::Timeout;
        bool reverse = false;
        int noProgressMs = 0;
        int noLookMs = 0;

        if (dir == PursuitDir::Reverse) reverse = true;
        else if (dir == PursuitDir::Forward) reverse = false;
        else {
            double dxEnd = path.back().first - pose.x;
            double dyEnd = path.back().second - pose.y;
            double h = deg2rad(pose.theta);
            double xREnd = dxEnd * std::cos(h) + dyEnd * std::sin(h);
            reverse = (xREnd < -2.0);
        }

        while (pros::millis() - startTime < timeoutMs) {
            if (cancelRequested.load() || pros::competition::is_disabled()) {
                result = FollowResult::Cancelled;
                break;
            }

            double endDist = std::hypot(path.back().first - pose.x, path.back().second - pose.y);
            if (endDist < pursuit.stopDist) {
                result = FollowResult::ReachedEnd;
                break;
            }

            double nHeading = reverse ? wrapDeg(pose.theta + 180.0) : pose.theta;

            double lookX, lookY;
            bool hasLook = findLookaheadPoint(path, Ld, lastSeg, nHeading, pursuit.forwardTol, lookX, lookY);
            if (!hasLook) noLookMs += 10;
            else noLookMs = 0;

            // Progress is measured by how far along the path (segment index) the lookahead search has advanced, not by Euclidean distance to
            // the final point, that distance can stay the same or even rise while the robot is correctly moving along the path
            if (progressCheck) {
                if (lastSeg > bestProgressSeg) {
                    bestProgressSeg = lastSeg;
                    noProgressMs = 0;
                } else {
                    noProgressMs += 10;
                }
                if (noProgressMs >= pursuit.stuckMs) {
                    result = FollowResult::Stuck;
                    break;
                }
            }

            double dx = lookX - pose.x;
            double dy = lookY - pose.y;
            double h = deg2rad(nHeading);
            double cosT = std::cos(h);
            double sinT = std::sin(h);
            double xR = dx * cosT + dy * sinT;
            double yR = -dx * sinT + dy * cosT;
            if (xR < 1.0) xR = 1.0;

            if (!hasLook && xR < pursuit.forwardTol && noLookMs >= pursuit.noLookFailMs) {
                size_t closest = lastSeg;
                double bestD = 1e9;
                size_t start = (lastSeg > 15) ? lastSeg - 15 : 0;
                size_t end = std::min(path.size(), lastSeg + 25);
                for (size_t i = start; i < end; ++i) {
                    double ddx = path[i].first - pose.x;
                    double ddy = path[i].second - pose.y;
                    double d = ddx * ddx + ddy * ddy;
                    if (d < bestD) { bestD = d; closest = i; }
                }
                driveToPoint(path[closest].first, path[closest].second, maxVolt, false);
                lastSeg = (closest > 0) ? closest - 1 : 0;
            }

            // Curvature: standard pure-pursuit control law (curvature = 2y/L^2
            // in the robot frame). `kRaw` is the true steering demand; `k` is
            // the version clamped for wheel-speed synthesis. The speed
            // slowdown below is deliberately computed from `kRaw`, not `k` —
            // using the clamped value would under-slow exactly on the
            // sharpest turns, the ones that need it most.
            double L = std::hypot(xR, yR);
            double denom = hasLook ? (Ld * Ld) : std::max(1.0, (L * L));
            double curvature = (2.0 * yR) / denom;
            double kRaw = curvature * (motors.getTrackWidth() / 2.0);
            double k = clampd(kRaw, -pursuit.curvClampK, pursuit.curvClampK);

            double base = maxVolt;
            if (endDist < pursuit.endSlowDist) {
                double frac = pursuit.endSlowMinFrac + (1.0 - pursuit.endSlowMinFrac) * (endDist / pursuit.endSlowDist);
                base = maxVolt * clampd(frac, pursuit.endSlowMinFrac, 1.0);
            }
            double curveFrac = 1.0 - std::min(0.70, std::fabs(kRaw) * pursuit.curvSlowK);
            base *= curveFrac;
            base = clampd(base, pursuit.minBase, maxVolt);

            double sign = reverse ? -1.0 : 1.0;
            double leftV = base * (1.0 - sign * k);
            double rightV = base * (1.0 + sign * k);
            leftV *= sign;
            rightV *= sign;

            leftV = limitOutput(leftV, prevL, -maxVolt, maxVolt, slewRate);
            rightV = limitOutput(rightV, prevR, -maxVolt, maxVolt, slewRate);

            motors.setVoltage(leftV, rightV);
            pros::Task::delay_until(&prevTime, stepMs);
        }

        motors.stop(BrakeMode::Brake);
        return result;
    }
 
    void tuneOffset() {
        requestMotion();
        double SdV_ccw = 0, SdV_cw = 0;//Sum of delta Vertical over turns
        double SdH_ccw = 0, SdH_cw = 0;
        double Stheta_ccw = 0, Stheta_cw = 0;

        //Need radians for formula
        auto spin = [&] (double lSpeed, double rSpeed, double angle, 
                        double& SdV, double& SdH, double& Stheta) {

            double H_IN_PER_TICK = (odom.horizontalDiameter * M_PI) / 36000.0;
            double V_IN_PER_TICK = (odom.verticalDiameter * M_PI) / 36000.0;
            double radAngle = deg2rad(angle);

            double lastHReading = odom.horizontal->get_position(), lastVReading = odom.vertical->get_position(), lastTheta = pose.theta;
            motors.left->move_voltage(lSpeed);
            motors.right->move_voltage(rSpeed);
            pros::delay(200);
            
            while (fabs(Stheta) < radAngle) {
                double currentHReading = odom.horizontal->get_position(), currentVReading = odom.vertical->get_position(), currentTheta = pose.theta;
                double thetaDeg = angleDiffDeg(currentTheta, lastTheta);

                SdV += (currentVReading - lastVReading) * V_IN_PER_TICK;
                SdH += (currentHReading - lastHReading) * H_IN_PER_TICK;
                Stheta += deg2rad(thetaDeg);

                lastTheta = currentTheta;
                lastHReading = currentHReading;
                lastVReading = currentVReading;
                pros::delay(10);
            }
                // motors.left->move_voltage(0);
                // motors.right->move_voltage(0);
            pros::delay(300);
        };
        spin(-7000, 7000, 720.0, SdV_ccw, SdH_ccw, Stheta_ccw);
        motors.left->move_voltage(0);
        motors.right->move_voltage(0);
        pros::delay(1000);
        spin(7000, -7000, 720.0, SdV_cw, SdH_cw, Stheta_cw);


        double denom = (Stheta_ccw - Stheta_cw);
        double horizontalOffset = (SdH_ccw - SdH_cw) / denom;
        double verticalOffset = (SdV_ccw - SdV_cw) / denom;
        pros::delay(200);
        printf("Vertical Offset: %.3lf\n", verticalOffset);
        pros::delay(200);
        printf("Horizontal Offset: %.3lf\n", horizontalOffset);
        motors.left->move_voltage(0);
        motors.right->move_voltage(0);
        pros::delay(1000);
        endMotion();
    }

    void waitUntilDone(){
        do pros::delay(25);
        while (motionRunning.load());
    }
    
    void waitUntil(double target){
        do pros::delay(25);
        while (motionRunning.load() && progress.load() < target);
    }

    void waitUntilPhase(int n){
        do pros::delay(25);
        while (motionRunning.load() && phase.load() < n);
    }
private:
    // No gains by default (no correction) until setHeadingCorrectionGains() is called.
    PID angularPID{0.0, 0.0, 0.0, 0.0}; // Heading correction
    PID turnPID; // For turn functions
    PID drivePID; // For drive
    PID swingPID; // For swing turns 
    Settle driveSettle;// For drives
    Settle turnSettle; // For turns
    Settle swingSettle; //For swinging turns

    Pose pose{0.0, 0.0, 0.0};
    double lastVertTicks = 0.0;
    double lastHorizTicks = 0.0;
    double lastHeadingDeg = 0.0;

    bool odomTaskStarted = false;
    std::atomic<bool> cancelRequested{false};

    // //Asymetrical slew
    double slew(double target, double prev, double maxDelta) const{
        double delta = target - prev;
        if (fabs(target) > fabs(prev) && fabs(delta) > maxDelta) {
            // Limit acceleration toward the target without overshooting it
            return prev + copysign(maxDelta, delta);
        }
        return target;  // Decel or within step; allow full change
    }

    // Applies battery-voltage scaling, a slew-rate limit, and clamping — the
    // sequence every motor command in this class goes through before it's sent.
    double limitOutput(double v, double& prev, double minV, double maxV, double slewRate) const {
        double scale = batteryScale();
        v = clampd(v * scale, minV, maxV);
        v = slew(v, prev, slewRate);
        v = clampd(v, minV, maxV);
        prev = v;
        return v;
    }

    static double inchesPerTick(double wheelDiamIn) {
        return (wheelDiamIn * M_PI) / 36000.0;
    }

    // Multi-IMU heading average, converted from compass to math convention.
    // Skips NaN/inf readings; falls back to the last known heading if every
    // IMU is unreadable.
    double readHeadingDeg() const {
        double sumSin = 0.0, sumCos = 0.0;
        int n = 0;
        for (auto* imu : odom.imus) {
            double raw = imu->get_heading();
            if (std::isnan(raw) || std::isinf(raw)) continue;
            double mathDeg = wrapDeg(90.0 - raw);
            sumSin += std::sin(deg2rad(mathDeg));
            sumCos += std::cos(deg2rad(mathDeg));
            ++n;
        }
        if (n == 0) return pose.theta;
        return wrapDeg(rad2deg(std::atan2(sumSin, sumCos)));
    }

    void odomLoop() {
        const int stepMs = 10;
        std::uint32_t prevTime = pros::millis();
        while (true) {
            updateOdometry(stepMs / 1000.0);
            pros::Task::delay_until(&prevTime, stepMs);
        }
    }

    void updateOdometry(double /*dt*/) {
        const double V_IN_PER_TICK = inchesPerTick(odom.verticalDiameter);
        const double H_IN_PER_TICK = inchesPerTick(odom.horizontalDiameter);

        double vNow = odom.vertical ? odom.vertical->get_position() : lastVertTicks;
        double hNow = odom.horizontal ? odom.horizontal->get_position() : lastHorizTicks;
        if (std::isnan(vNow) || std::isinf(vNow)) vNow = lastVertTicks;
        if (std::isnan(hNow) || std::isinf(hNow)) hNow = lastHorizTicks;

        double headDeg = odom.imus.empty() ? pose.theta : readHeadingDeg();
        double dThetaDeg = angleDiffDeg(headDeg, lastHeadingDeg);
        double dTheta = deg2rad(dThetaDeg);

        double dV = (vNow - lastVertTicks) * V_IN_PER_TICK;
        double dH = (hNow - lastHorizTicks) * H_IN_PER_TICK;

        double forward = dV - (odom.verticalOffset * dTheta);
        double left = dH - (odom.horizontalOffset * dTheta);

        double mid = deg2rad(lastHeadingDeg) + dTheta * 0.5;
        double cosT = std::cos(mid);
        double sinT = std::sin(mid);

        pose.x += forward * cosT - left * sinT;
        pose.y += forward * sinT + left * cosT;
        pose.theta = headDeg;

        lastVertTicks = vNow;
        lastHorizTicks = hNow;
        lastHeadingDeg = headDeg;
    }

    // Circle-segment intersection; returns the intersection further along the segment.
    static bool circleSegmentIntersection(double cx, double cy, double r,
                                           double x1, double y1, double x2, double y2,
                                           double& ix, double& iy, double& tOut) {
        double dx = x2 - x1, dy = y2 - y1;
        double fx = x1 - cx, fy = y1 - cy;

        double a = dx * dx + dy * dy;
        if (a < 1e-9) return false;

        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;

        double disc = b * b - 4 * a * c;
        if (disc < 0) return false;
        disc = std::sqrt(disc);

        double t1 = (-b - disc) / (2 * a);
        double t2 = (-b + disc) / (2 * a);
        bool ok1 = (t1 >= 0.0 && t1 <= 1.0);
        bool ok2 = (t2 >= 0.0 && t2 <= 1.0);
        if (!ok1 && !ok2) return false;

        double t = ok1 && ok2 ? std::max(t1, t2) : (ok1 ? t1 : t2);
        ix = x1 + t * dx;
        iy = y1 + t * dy;
        tOut = t;
        return true;
    }

    // Finds the lookahead point by intersecting a circle around the robot with
    // path segments, preferring the intersection furthest along the path and
    // rejecting points behind the robot. If no intersection is found, falls
    // back to a point reached by walking forward along the path from the
    // nearest waypoint until roughly `lookaheadDist` of arc length has been
    // covered (or the path runs out) — never an unconstrained "next point"
    // that could sit right on top of the robot and produce noisy curvature.
    bool findLookaheadPoint(const std::vector<std::pair<double, double>>& path,
                             double lookaheadDist, size_t& lastSeg, double nHeading,
                             double forwardTol, double& lookX, double& lookY) const {
        const double cx = pose.x, cy = pose.y, r = lookaheadDist;

        bool found = false;
        size_t bestSeg = 0;
        double bestT = -1.0;
        size_t closest = lastSeg;
        double bestD = 1e9;

        size_t start = (lastSeg > 15) ? lastSeg - 15 : 0;
        size_t end = std::min(path.size(), lastSeg + 25);
        for (size_t i = start; i < end; ++i) {
            double dx = path[i].first - cx;
            double dy = path[i].second - cy;
            double d = dx * dx + dy * dy;
            if (d < bestD) { bestD = d; closest = i; }
        }

        double bestX, bestY;
        {
            size_t idx = closest;
            double px = path[idx].first, py = path[idx].second;
            double accum = std::hypot(px - cx, py - cy);
            while (accum < lookaheadDist && idx + 1 < path.size()) {
                double nx = path[idx + 1].first, ny = path[idx + 1].second;
                accum += std::hypot(nx - px, ny - py);
                ++idx;
                px = nx;
                py = ny;
            }
            bestX = px;
            bestY = py;
        }

        auto findFurthestPoint = [&](size_t i, double ix, double iy, double t) {
            double dx = ix - cx, dy = iy - cy;
            double h = deg2rad(nHeading);
            double xR = dx * std::cos(h) + dy * std::sin(h);
            if (xR < forwardTol) return;
            if (!found || i > bestSeg || (i == bestSeg && t > bestT)) {
                found = true;
                bestSeg = i;
                bestT = t;
                bestX = ix;
                bestY = iy;
            }
        };

        auto scan = [&](size_t s, size_t e) {
            for (size_t i = s; i < e; ++i) {
                double x1 = path[i].first, y1 = path[i].second;
                double x2 = path[i + 1].first, y2 = path[i + 1].second;
                double ix, iy, t;
                if (!circleSegmentIntersection(cx, cy, r, x1, y1, x2, y2, ix, iy, t)) continue;
                findFurthestPoint(i, ix, iy, t);
            }
        };

        if (path.size() >= 2) {
            scan(lastSeg, path.size() - 1);
            if (!found && lastSeg != 0) scan(0, lastSeg);
        }

        lookX = bestX;
        lookY = bestY;

        if (found) {
            lastSeg = bestSeg;
            return true;
        }
        lastSeg = closest;
        return false;
    }
    
    //Async variables
    pros::RecursiveMutex mutex;            
    std::atomic<bool>   motionRunning{false};
    std::atomic<double> progress{0.0};
    std::atomic<int>    phase{0};
    bool progressPaused = false;
    int  motionDepth = 0;    

    void requestMotion() {
        mutex.take(TIMEOUT_MAX);
        if (motionDepth++ ==0){
            progress = 0.0;
            phase = 0;
            progressPaused = false;
            cancelRequested = false;
            motionRunning = true;
        }
    }
    
    void endMotion(){
       if(--motionDepth == 0) motionRunning = false;
       mutex.give();
    }

};

} // namespace yogurt
