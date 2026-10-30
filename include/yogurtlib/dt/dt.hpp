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

    /**
     * @brief Set the PID values for heading correction with the driveStraight movement, used for maintaing straight drives or intentional angling
     * 
     * Tune until drive is consistently straight and the final heading is within 0.2 degress of the start
     *
     * @param kP mV per unit of error within a 10 ms timespan
     * @param kI mV based on accumulated error over time; Only use if you dseire to hold a position for long periods of time
     * @param kD mV per unit of error to counteract kP over time
     * @param maxIntergral the maximum windup of kI in mV
     *
     * ### Example Code:
     * 
     *```cpp
     * void intitalize(){
     *    drivetrain.setHeadingCorrectionGains(120.0, 0.0, 6.0, 5.0);
     * }
     * ```
    */
    void setHeadingCorrectionGains(double kp, double ki, double kd, double maxIntegral);
    
    /**
     * @brief Set the settle values for Boomerang movements, both driving and turning
     * 
     * Tune the values to reflect how presise you want your movement to be
     *
     * @param driveTolerance tolerance in inches the robot is allowed to be off
     * @param driveVelocity how fast the driving can be going
     * @param driveMs how long driving tolerance and velocity must to meet their values for the driving to settle
     * @param turnTolerance tolerance in degress the robot is allowed to be off
     * @param turnVelocity how fast the robot can be turning
     * @param turnMs how long turning tolerance and velocity must meet their values for the turning to settle
     *
     * ### Example Code:
     * 
     * ```cpp
     * void intitalize(){
     *    drivetrain.setBoomerangSettleGains({1.0, 2.5, 250}, {1.5, 8.0, 250});
     * }
     *
    */
    void setBoomerangSettleGains(Settle drive = {1.0, 2.5, 250}, Settle turn = {1.5, 8.0, 250});

    // ---- Setup ----
    std::vector<std::function<void(Drivetrain&)>> onCalibrated;

    /**
     * @brief Calibrate the imus, reset the pose and start the odom task
     * 
     * Call me once in initalize
     *
     * @param x the X value of the center of the robot
     * @param y the Y value of the center of the robot
     * @param heading the heading the robot faces in math standard (90 degrees = north, 0 degrees = east)
     *
     * ### Example Code:
     * 
     * ```cpp
     * void intitalize(){
     *    drivetrain.calibrate({0.0, 0.0, 90.0});
     * }
     *
    */
    void calibrate(Pose startPose = Pose(0.0, 0.0, 90.0));

    /**
     * @brief cancels the current motion on its next loop, and allows next async motion to run
     * 
     * Also safe to call in any other functions (Such as disabled section to ensure the robot stops moving)
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.motion(a, b, c);
     *    drivetrain.cancelMotion();
     * }
     *
    */
    void cancelMotion();
    
    /**
    * @brief returns the drivetrais track width, used to pass the trackwidth into functions that require it (such as PurePursuit)
    */
    double getTrackWidth() const;

    /**
    * @brief stops the motor movement
    * 
    * WARNING, THIS WILL PHYSICALLY STOP THE MOTORS BUT NOT STOP THE FUNCTIONS THAT ARE CONTROLLING THE MOTORS AT THE TIME IT IS RAN, TO STOP THE FUNCTION , CALL 
    *
    * #### cancelMotion()
    * 
    * Errorenous use of this function may cause your program to freeze
    */
    void stop();

    // ---- Pose ----

    /**
     * @brief returns the current pose
    */
    Pose getPose() const;
    
    /**
     * @brief set the pose of the robot, call at the start of each autonomus routine
     *
     * @param x the x pose of the robot
     * @param y the y pose of the robot
     * @param heading the heading the robot faces in math standard (90 degrees = north, 0 degrees = east)
     * 
     * ### Example Code
     * ```cpp
     * void autonomus() {
     *    setPose({-12.0, 24.0, 0.0});
     * }
    */
    void setPose(const Pose& p);

    // ---- Motions ----

    /**
     * @brief Drive straight for a certian distance, with a controllable angular offset
     *
     * @param distanceInches how far the robot is to drive, positive is forwards, negitive is backwards
     * @param timeoutMs how long the movement can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param headingOffset the angle the robot should drive at relitive to the current heading
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.driveStraight(10.0, 1000, {.maxVolt = 10000.0, .headingOffset = 10.0});
     * }
     *
    */
    MotionResult driveStraight(double distanceInches, int timeoutMs, DriveParams params = {}, bool async = false);

    /**
     * @brief turns the robot in place to face a desired feild oriented heading
     *
     * @param targetDeg the heading the robot should turn to face
     * @param timeoutMs how long the movement can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.turnToHeading(45.0, 1000, {.minVolt = 2000, .earlyExitRange = 10.0});
     * }
     *
    */
    MotionResult turnToHeading(double targetDeg, int timeoutMs, TurnToHeadParams params = {}, bool async = false);

    /**
     * @brief turns the robot in place to face a desired point on the feild
     *
     * @param x the x position of the point
     * @param y the y position of the point
     * @param timeoutMs how long the movement can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param reversed if the front or back of the robot should turn to face the point (True = back, false = front)
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.turnToPoint(-48.0, 24.0, 1000, {.reversed = true}, true);
     * }
     *
    */
    MotionResult turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params = {}, bool async = false);

    /**
     * @brief swings around one side of the drivetrain to a desired heading
     *
     * @param targetDeg the desired final heading of the robot
     * @param timeoutMs how long the movement can run for before it is canceled if it does not settle
     * @param leftSidePivot the side of the drivetrain to pivot around (true = left, false = right)
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.swingToHeading(270.0, 1000, false);
     * }
     *
    */
    MotionResult swingToHeading(double targetDeg, int timeoutMs, bool leftSidePivot = true, SwingToHeadingParams params = {}, bool async = false);

    /**
     * @brief preforms a drive and a turn at the same time, requires boomerang settle to be set
     *
     * @param x the desired final x
     * @param y the desired final y
     * @param heading the desired final heading
     * @param timeoutMs how long the movement can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param lead how far to project the carrot point behind the target point (between 0 and 1)
     * @param curveK how much to slow down for curving
     * @param closeDist how close to the desired pose the robot will start a drive directly towards it
     * @param reversed if the front or back of the robot should turn to face the point (True = back, false = front)
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.boomerang(-48.0, 24.0, 132.0, {.lead = 0.5, .closeDist = 0.2});
     * }
     *
    */
    MotionResult boomerang(double x, double y, double heading, double timeOutMs, BoomParams params = {}, bool async = false);

    /**
     * @brief Drive to a point on the feild
     *
     * @param x the x position of the point
     * @param y the y position of the point
     * @param turnTimeout how long the turn can run for before it is canceled if it does not settle
     * @param driveTimeout how long the drive can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param usePurePursuit if pure pursuit should be used (if true, drive and turn timeout is combined, if false, a turn than drive is ran)
     * @param reversed if the robot should drive forwards or reversed (True = reversed, false = forwards)
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.driveToPoint(-48.0, 34.0, 1000, 1500, {.usePurePursuit = false});
     * }
     *
    */
    MotionResult driveToPoint(double x, double y, int turnTimeout, int driveTimeout, DriveToPointParams params = {}, bool async = false);


    /**
     * @brief Drive to a point on the feild and than turn to a heading
     *
     * @param x the x position of the pose
     * @param y the y position of the pose
     * @param headingDeg the heading of the pose
     * @param turnTimeout how long the turn can run for before it is canceled if it does not settle
     * @param driveTimeout how long the drive can run for before it is canceled if it does not settle
     * @param maxVolt maximum speed in mV (0 - 12000) the robot can acclerate to during the motion
     * @param minVolt the minimum speed in mV the robot will slow down to when decelerating
     * @param usePurePursuit if pure pursuit should be used (if true, drive and turn timeout is combined, if false, a turn than drive is ran)
     * @param reversed if the robot should drive forwards or reversed (True = reversed, false = forwards)
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minVolt need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     * @param async if the motion should run asyncronusly and be added to the motion queue or wait for the last motion to finish
     *
     * ### Example Code:
     * 
     * ```cpp
     * void autonomus(){
     *    drivetrain.driveToPose(-48.0, 34.0, 70.0, 1000, 1500, {.reversed = true});
     * }
     *
    */
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
        };
        spin(-7000, 7000, 720.0, SdV_ccw, SdH_ccw, Stheta_ccw);
        motors.left->move_voltage(0);
        motors.right->move_voltage(0);
        pros::delay(1000);
        spin(7000, -7000, 720.0, SdV_cw, SdH_cw, Stheta_cw);


        double denom = (Stheta_ccw - Stheta_cw);
        double horizontalOffset = (SdH_ccw - SdH_cw) / denom;
        double verticalOffset = (SdV_ccw - SdV_cw) / denom;
        printf("Vertical Offset: %.3lf\nHorizontal Offset: %.3lf\n", verticalOffset, horizontalOffset);
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
    Settle boomerangDriveSettle{1.0, 2.5, 250}; //For boomerang drive motions
    Settle boomerangTurnSettle{1.5, 8.0, 250};  //For boomerang turn motions

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

    void requestMotion();
    
    void endMotion();

};

} // namespace yogurt