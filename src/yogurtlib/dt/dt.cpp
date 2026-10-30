#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

ControllerSettings::ControllerSettings(double p, double i, double d, double maxInt,
                        double errTol, double velTol, int settleMs)
                        : kp(p), ki(i), kd(d), maxIntegral(maxInt),
                        errTol(errTol), velTol(velTol), settleMs(settleMs) {} 

OdomSensors::OdomSensors(pros::Rotation* vertical, double verticalDiameter, double verticalOffset,
                         pros::Rotation* horizontal, double horizontalDiameter, double horizontalOffset,
                         yogurt::IMU imus, pros::Distance* leftD, pros::Distance* frontD,
                         pros::Distance* rightD, pros::Distance* backD, double leftOffset, double rightOffset,
                         double frontOffset, double backOffset) :
                         vertical(vertical), verticalDiameter(verticalDiameter), verticalOffset(verticalOffset),
                         horizontal(horizontal), horizontalDiameter(horizontalDiameter), horizontalOffset(horizontalOffset),
                         imus(imus), leftD(leftD), frontD(frontD), rightD(rightD), backD(backD),
                         leftOffset(leftOffset), rightOffset(rightOffset), frontOffset(frontOffset), backOffset(backOffset) {}

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
    for (auto* imu : odom.imus.sensors) {
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

void Drivetrain::cancelMotion() { cancelRequested = true; }

double Drivetrain::getTrackWidth() const { return trackWidth; }

void Drivetrain::stop() { motors.stop(toBrakeMode(motors.left->get_brake_mode())); }

Pose Drivetrain::getPose() const { return pose; }

void Drivetrain::setPose(const yogurt::Pose& p) {
    pose = p;

    double compassHeading = wrapDeg(90.0 - pose.theta);
    if (compassHeading < 0) compassHeading += 360.0;
    for (auto* imu : odom.imus.sensors) imu->set_heading(compassHeading);

    if (odom.vertical) lastVertTicks = odom.vertical->get_position();
    if (odom.horizontal) lastHorizTicks = odom.horizontal->get_position();
    lastHeadingDeg = pose.theta;
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
    mutex.take(TIMEOUT_MAX);
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
    mutex.give();
}