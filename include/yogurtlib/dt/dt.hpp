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

/**
 * @brief robot setting for PurePursuit, used across all instances of drivetrain.curve()
 * 
 * retune per-robot on the Drivetrain instance, not by editing this header
 * 
 * @param voltTerms voltage terms for pure pursuit, similar but also very different to a PID, see yogurt::VoltageTerms for documentation
 * @param lookaheadDist default lookahead, while tuneable per call, if a value is not given per-call it will revert to this value
 * @param endTolerance counts a movement as done once within this amount in inches
 * @param stuckMs how long the robot doesnt move by stuckDist for to assume it is stuck
 * @param stuckDist how far the robot must move in stuckMs to not be stuck in inches
 * @param lostDist how far from the path the robot must be to assume it is lost in inches
 * @param speedAlpha how much to smooth the measured speed based on drivetrain motor encoders
 * 
 * ### Example Code
 * 
 * ```cpp
 * void intialize() {
 *    drivetrain.PursuitSettings = ({.voltTerms = terms, .lookaheadDist = 12.0, .endTolerance = 2.0});
 *  }
 */
struct PursuitProfile {
    VoltageTerms voltTerms;
    double lookaheadDist = 12.0;
    double endTolerance = 1.0;
    int stuckMs = 500;
    double stuckDist = 0.5;
    double lostDist = 12.0; 
    double speedAlpha = 0.4;
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
     * @param turnTimeout how long the motion can run for before it is canceled if it does not settle
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
     * @param turnTimeout how long the motion can run for before it is canceled if it does not settle
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

    /**
     * @brief follows a curved or straight path
     * 
     * The curve motion requires:
     * 
     * - a path in a yogurtlib path vector (yogurt::pathVec)
     * 
     * - Pathconfigs (see yogurt::Pathconfig for documentation)
     * 
     * - Pursuit Settigns (see drivetrain.PursuitSettings for documentation)
     * 
     * @param path the path that the robot should follow, aswell as the configs
     * @param timeoutMs how long the motion can run for before it is canceled if it does not settle
     * @param reversed if the drivetrain should drive forwards or reversed for the motion (false = forwards, true = reversed)
     * @param lookaheadDist how far ahead the robot should "look" before it thinks it is lost
     * @param minSpeed the minimum speed the robot will slow down to when decelerating (scale of 0 - 127)
     * @param earlyExitRange how close the the end a motion chain can be activated, Both this and minSpeed need values for a chain to be executed
     * @param slewRate the rate at which the motion can accelerate in mV per 10 ms to prevent wheel slip
     *  ### Example Code:
     * 
     * ```
     * void autonomus() {
     *   drivetrain.curve(loop, 2000, {.reversed = true});
     * } 
     * ```
     */
    MotionResult curve(Path path, int timeoutMs, CurveParams params = {}, bool async = false);

    /**
     * @brief gets the actualSpeed of the drivetrain using motor encoder rpm's, helper function for purepursuit
     */
    double getActualSpeed (pros::MotorGroup& mg, double alpha, double& measured, double& lastTicks, std::uint32_t& lastStamp);

   /**
    * @brief takes in the speed from pure pursuit and converts to voltage to set the motors too, helper function for purepursuit
    */
    static double speedToVoltage (double vel, double accel, double measured, const VoltageTerms& vt);
 
    /**
     * @brief run this to tune the offset of odom pods, should be done after odom wheel diameters are tuned, ideally ran multiple times with the values averaged, especially helpful for all omniwheel set ups
     */
    void tuneOffset();

    //Asymetrical slew
    static double slew(double target, double prev, double maxDelta);

    /**
     * @brief wait until the previous asyncronusly run function finishes, useful if you want to change pneumatics/etc while moving but dont want to start scoring/etc
     * 
     * ### Example Code
     * 
     * ```cpp
     * void autonomus() {
     *    drivetrain.motion(a, b, c, true);
     *    drivetrain.waitUntilDone();
     * }
     */
    void waitUntilDone();
    
    /**
     * @brief a wait function for async functions
     * 
     * @param target the target that should be waited until, units depend on the motion that is running asyncronusly (Inches or degress)
     * 
     * ### Example Code
     * 
     * ```cpp
     * void autonomus() {
     *   drivetrain.motion(a, b, c, true);
     *   drivetrain.waitUntil(10.0);
     * }
     */
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