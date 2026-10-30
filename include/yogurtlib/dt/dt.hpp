#pragma once
#include "main.h"
#include <atomic>
#include <functional>
#include "yogurtlib/control.hpp"
#include "yogurtlib/odometry.hpp"
#include "yogurtlib/pose.hpp"
#include "yogurtlib/dt/motors.hpp"
#include "yogurtlib/pursuit.hpp"
#include "yogurtlib/misc.hpp"

namespace yogurt {

// Bundles a PID's gains with the Settle config used to decide when a motion
// driven by that PID has finished.
struct ControllerSettings {
    double kp, ki, kd, maxIntegral;
    double errTol, velTol;
    int settleMs;

    ControllerSettings(double p, double i, double d, double maxInt,
                        double errTol, double velTol, int settleMs);
    
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

    IMU imus;

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
                yogurt::IMU imus, pros::Distance* leftD, pros::Distance* frontD,
                pros::Distance* rightD, pros::Distance* backD, double leftOffset, double rightOffset,
                double frontOffset, double backOffset);

};

// Tunable pure-pursuit profile. Defaults are reasonable starting points —
// retune per-robot on the Drivetrain instance, not by editing this header.
struct PursuitProfile {
    VoltageTerms voltTerms; //Tuneable values for pure pursuit ("PID")
    double lookaheadDist = 12.0; //How far ahead we look for points
    double endTolerance = 1.0; //Counts as done once within this amount from the end point (Inches)
    int stuckMs = 500; //if no path progress for this long -> Stuck
    double stuckDist = 0.5; //If progressS doesnt increase by this  much we assume stuck
    double lostDist = 12.0; //Exit if the robot is further than this from the nearest path point
    double speedAlpha = 0.4; //Smoothing for the measured speed
};

struct DriveParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    double headingOffset = 0.0;
    double earlyExitRange = 0.0;
    double slewRate = -1.0; 
};

struct TurnToHeadParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    double earlyExitRange = 0.0;
    double slewRate = -1.0; 
};

struct TurnToPointParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    bool reversed = false;
    double earlyExitRange = 0.0;
    double slewRate = -1.0;
};

struct SwingToHeadingParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    double earlyExitRange = 0.0;
    double slewRate = -1.0;
};

struct BoomParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    double lead = 0.6;
    double curveK = 0.0; //Tune if your robot is drifting out too much sideways during call
    double closeDist = 7.0;
    bool reversed = false;
    double earlyExitRange = 0.0;
    double slewRate = -1.0;
};

struct DriveToPointParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    bool usePurePursuit = true; 
    bool reversed = false;
    double earlyExitRange = 0.0;
    double slewRate = -1.0;
};

struct DriveToPoseParams {
    double maxVolt = -1.0;
    double minVolt = 0.0;
    bool usePurePursuit = true;
    bool reversed = false;
    double earlyExitRange = 0.0;
    double slewRate = -1.0;
};

struct CurveParams {
    bool reversed = false;
    double lookaheadDist = -1.0; // Tunable per call, otherwise uses default value in Pursuit Profile
    double minSpeed = 0.0; // Out of 127, keeps moving at end for motion chaining
    double earlyExitRange = 0.0; //When to move on to the next function when motion chaining so doens't overshoot
    double slewRate = -1.0; // Tunable per call, otherwise uses default value (600mv per loop)
};


// Outcome of a blocking point/turn motion (driveStraight, turnToHeading,
// turnToPoint, swingToHeading).
enum class MotionResult {
    Settled,
    Timeout,
    Cancelled,
    Running,
    Chained,
    Stuck,
    Lost,
    InvalidPath
};

inline std::string entostr (MotionResult result) {
    int num = (int)result;
    switch (num){
        case 0  : return "Settled";
        case 1  : return "Timeout";
        case 2  : return "Cancelled";
        case 3  : return "Running";
        case 4  : return "Chained";
        case 5  : return "Stuck";
        case 6  : return "Lost";
        case 7  : return "InvalidPath";
        default : return "Unknown";
    }
}



// Top-level drivetrain object: owns the drive hardware, PID controllers,
// pose estimate, and every drive/turn/path-following motion. This is the
// object autonomous code should talk to; it replaces the free functions in
// drive.cpp/odometry.cpp with one cohesive class.
class Drivetrain {
public:
    DriveMotors motors;
    OdomSensors odom;
    PursuitProfile pursuitSettings;
    PathConfig pathSettings;
    
    //Default maxVoltage and slew
    double maxVoltage = 12000.0;
    double defaultSlew = 600.0;

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
    double swingKsThreshold = 1.5;

    double boomerangCloseMinVolt = 5670.0;
    double driveToMinVolt = 3000.0; //210E NEEDS TO TUNE THIS!
    std::pair<double, double> driveToEarlyExit = {7.0, 4.0}; //TUNE THIS ALSO!!!

    // Only the pieces with no sane default are constructor arguments: the
    // physical hardware, and the three gain sets a Drivetrain can't do its
    // job without (drive/turn/swing). Everything else above is a settable
    // field with a working default, or configured via a setter below
    // (see setHeadingCorrectionGains) — call those after construction if you
    // want non-default behavior, instead of one long argument list.
    Drivetrain(DriveMotors motors, OdomSensors odomSensors,
               ControllerSettings driveSettings, ControllerSettings turnSettings, //Drive for driveStraight, turn for turnning
               ControllerSettings swingSettings); //swing for sing settings

    // PID for setting the heading correction gains in driveStraight 
    // for maintaining a straight path(or intentional angling)
    // Call it once during setup if you want driveStraight to hold heading. 
    // Tune values before hand until drive is consistently straight and final heading it within 0.2 degrees of the start.
    void setHeadingCorrectionGains(double kp, double ki, double kd, double maxIntegral);
    
    void setBoomerangSettleGains(Settle drive = {1.0, 2.5, 200}, Settle turn = {1.5, 8.0, 200});

    // ---- Setup ----
    std::vector<std::function<void(Drivetrain&)>> onCalibrated;

    // Blocking IMU reset + starts the odometry task. Call once from initialize().
    void calibrate(Pose startPose = Pose(0, 0, 90));

    // Requests that any in-progress blocking motion stop at its next
    // iteration. Safe to call from another task (e.g. a disable handler).
    void cancelMotion();
    
    //Gets the track width
    double getTrackWidth() const;
    
    //Warning, this will physically stop the motors but will not kill the functions running the motors
    //To cancel the running of the functions call cancelMotion()
    //Errorenous use of this function could cause your program to freeze
    void stop();

    // ---- Pose ----

    Pose getPose() const;

    void setPose(const Pose& p);

    // ---- Motions ----

    // Drives a length in inches along the current (or offset) heading, holding
    // heading with the angular PID. Positive distance drives forward.
    MotionResult driveStraight(double distanceInches, int timeoutMs, DriveParams params = {}, bool async = false);

    // Turns in place to face `targetDeg`.
    MotionResult turnToHeading(double targetDeg, int timeoutMs, TurnToHeadParams params = {}, bool async = false);

    // Turns in place to face the point (x, y).
    MotionResult turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params = {}, bool async = false);

    // Pivots about one side only (the other side stays stopped) to face `targetDeg`.
    MotionResult swingToHeading(double targetDeg, int timeoutMs, bool leftSidePivot = true, SwingToHeadingParams params = {}, bool async = false);

    MotionResult boomerang(double x, double y, double heading, double timeOutMs, BoomParams params = {}, bool async = false);

    // Drives to (x, y): pure pursuit by default, or turn-then-drive if
    // usePurePursuit is false.
    MotionResult driveToPoint(double x, double y, int turnTimeout, int driveTimeout, DriveToPointParams params = {}, bool async = false);


    // Drives to (x, y) then turns to face `headingDeg`.
    MotionResult driveToPose(double x, double y, double headingDeg, int turnTimeout, int driveTimeout, DriveToPoseParams params = {}, bool async = false);

    
    
    // Gets the actual rpm of the motors using encoder units
    double getActualSpeed (pros::MotorGroup& mg, double alpha, double& measured, double& lastTicks, std::uint32_t& lastStamp);


    // Takes a speed from Curve and sets it to a voltage for the DT to understand
    static double speedToVoltage (double vel, double accel, double measured, const VoltageTerms& vt);

    MotionResult curve(Path path, int timeoutMs, CurveParams params = {}, bool async = false);
 
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

    //Asymetrical slew
    static double slew(double target, double prev, double maxDelta);

    void waitUntilDone();
    
    void waitUntil(double target);

private:
    // No gains by default (no correction) until setHeadingCorrectionGains() is called.
    PID angularPID{0.0, 0.0, 0.0, 0.0}; // Heading correction
    PID turnPID; // For turn functions
    PID drivePID; // For drive
    PID swingPID; // For swing turns 
    Settle driveSettle;// For drives
    Settle turnSettle; // For turns
    Settle swingSettle; //For swinging turns
    Settle boomerangDriveSettle{1.0, 2.5, 200}; //For boomerang drive motions
    Settle boomerangTurnSettle{1.5, 8.0, 200};  //For boomerang turn motions

    Pose pose{0.0, 0.0, 0.0};
    double trackWidth = 0.0; // inches, center-to-center of left/right wheels
    double lastVertTicks = 0.0;
    double lastHorizTicks = 0.0;
    double lastHeadingDeg = 0.0;

    bool odomTaskStarted = false;
    std::atomic<bool> cancelRequested{false};

    // Applies battery-voltage scaling, a slew-rate limit, and clamping — the
    // sequence every motor command in this class goes through before it's sent.
    double limitOutput(double v, double& prev, double minV, double maxV, double slewRate) const;

    bool chainExit (double error, int dir, double minV, double earlyExitRange);

    static double inchesPerTick(double wheelDiamIn);

    // Multi-IMU heading average, converted from compass to math convention.
    // Skips NaN/inf readings; falls back to the last known heading if every
    // IMU is unreadable.
    double readHeadingDeg() const;


    void odomLoop();

    void updateOdometry(double /*dt*/);
    
    //Async variables
    pros::RecursiveMutex mutex;            
    std::atomic<bool>   motionRunning{false};
    std::atomic<double> progress{0.0};
    bool progressPaused = false;
    int  motionDepth = 0;    

    void requestMotion() {
        mutex.take(TIMEOUT_MAX);
        if (motionDepth++ == 0){
            progress = 0.0;
            progressPaused = false;
            cancelRequested.store(false);
            motionRunning = true;
        }
    }
    
    void endMotion(){
        std::cout << "endMotion\n";
       if(--motionDepth == 0) motionRunning = false;
       mutex.give();
    }

};

} // namespace yogurt