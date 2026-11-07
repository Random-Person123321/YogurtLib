#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/misc.hpp"
using namespace yogurt;

OdomSensors::OdomSensors(pros::Rotation* vertical, double verticalDiameter, double verticalOffset,
                         pros::Rotation* horizontal, double horizontalDiameter, double horizontalOffset,
                         yogurt::IMU imus, pros::Distance* leftD, pros::Distance* frontD,
                         pros::Distance* rightD, pros::Distance* backD, double leftOffset, double frontOffset,
                         double rightOffset, double backOffset) :
                         vertical(vertical), verticalDiameter(verticalDiameter), verticalOffset(verticalOffset),
                         horizontal(horizontal), horizontalDiameter(horizontalDiameter), horizontalOffset(horizontalOffset),
                         imus(imus) {left.sensor = leftD, left.xOffset = leftOffset, front.sensor = frontD, front.yOffset = frontOffset,
                                    right.sensor = rightD, right.xOffset = rightOffset, back.sensor = leftD, back.yOffset = backOffset; }

Drivetrain::Drivetrain(DriveMotors motors, OdomSensors odomSensors,
                       ControllerSettings driveSettings, ControllerSettings turnSettings, //Drive for driveStraight, turn for turnning
                       ControllerSettings swingSettings) : //swing for sing settings
                       motors(motors), odom(odomSensors),
                       drivePID(driveSettings.kp, driveSettings.ki, driveSettings.kd, driveSettings.maxIntegral), //DriveStraight
                       turnPID(turnSettings.kp, turnSettings.ki, turnSettings.kd, turnSettings.maxIntegral), // Turning
                       swingPID(swingSettings.kp, swingSettings.ki, swingSettings.kd, swingSettings.maxIntegral), //Swing logic
                       driveSettle{driveSettings.errTol, driveSettings.velTol, driveSettings.settleMs}, // Drivestraight
                       turnSettle{turnSettings.errTol, turnSettings.velTol, turnSettings.settleMs}, //Settle for turning from turnSettings
                       swingSettle{swingSettings.errTol, swingSettings.velTol, swingSettings.settleMs} {}

void Drivetrain::setHeadingCorrectionGains(double kp, double ki, double kd, double maxIntegral) {
        angularPID = PID(kp, ki, kd, maxIntegral);
}

void Drivetrain::setBoomerangSettleGains(Settle drive, Settle turn) {
    boomerangDriveSettle = drive;
    boomerangTurnSettle = turn;
}

// Blocking IMU reset + starts the odometry task. Call once from initialize().
void Drivetrain::calibrate(Pose startPose) {
    if (!odom.imus.calibrate()) std::cout << "IMU failed Initialize\n";
    setPose(startPose);
    if (!odomTaskStarted) {
        odomTaskStarted = true;
        // The task keeps running independently of this handle's lifetime.
        pros::Task odomTask([this] { this->odomLoop(); });
    }
    for (auto& cb : onCalibrated) cb(*this); //Runs on the functions and passes the this pointer as part of the class in
}

void Drivetrain::cancelMotion() { cancelRequested = true; }

double Drivetrain::getTrackWidth() const { return trackWidth; }

void Drivetrain::stop() { motors.stop(toBrakeMode(motors.left->get_brake_mode())); }

Pose Drivetrain::getPose() const {
    poseMutex.take();
    Pose p = pose;
    poseMutex.give();
    p.theta = wrapDeg(p.theta);
    return p;
}

void Drivetrain::setPose(const yogurt::Pose& p) {
    poseMutex.take();
    double currentH = p.theta;
    pose.x = p.x;
    pose.y = p.y;
    
    pose.theta = currentH + angleDiffDeg(pose.theta, currentH);
    odom.imus.rebase();

    if (odom.vertical) lastVertTicks = odom.vertical->get_position();
    if (odom.horizontal) lastHorizTicks = odom.horizontal->get_position();
    lastHeadingDeg = pose.theta;
    poseMutex.give();
}

//Asymetrical slew
double Drivetrain::slew(double target, double prev, double maxDelta) {
    if (std::signbit(target) != std::signbit(prev) || target == 0.0) prev = 0.0; // A change of direction counts as braking
    double delta = target - prev; // A slowdown towards zero is unrestricted
    if (fabs(target) > fabs(prev) && fabs(delta) > maxDelta) {
        // Limit acceleration toward the target without overshooting it
        return prev + copysign(maxDelta, delta);
    }
    return target;  // Decel or within step; allow full change
}

void Drivetrain::waitUntilDone() {
    do pros::delay(20);
    while (motionRunning.load());
}

void Drivetrain::waitUntil(double target) {
    do pros::delay(20);
    while (motionRunning.load() && progress.load() < target);
}

// Async motion queuing functions
void Drivetrain::requestMotion() {
    asyncMutex.take(TIMEOUT_MAX);
    if (motionDepth++ == 0){
        progress = 0.0;
        progressPaused = false;
        cancelRequested.store(false);
        motionRunning = true;
    }
}

void Drivetrain::endMotion(){
    std::cout << "endMotion\n";
    if(--motionDepth == 0) motionRunning = false;
    asyncMutex.give();
}