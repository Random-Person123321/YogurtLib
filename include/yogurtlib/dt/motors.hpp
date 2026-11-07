#pragma once
#include "main.h"

namespace yogurt {

enum class BrakeMode {
    Coast,
    Brake,
    Hold
};

pros::motor_brake_mode_e_t toProsBrakeMode(BrakeMode mode);

BrakeMode toBrakeMode(pros::MotorBrake mode);

// Raw drivetrain specs
class DriveMotors {
public:
    pros::MotorGroup* left;
    pros::MotorGroup* right;

    /**
     * @brief The part of the drivetrain that owns the motors and is responsible for setting the voltages
     * 
     * @param left the motors on the left side of the drivetrain, setup in a PROS motor group
     * @param right the motors on the right side of the drivetrain, setup in a PROS motor group
     * @param wheelDiameter the diameter of the drivetrain wheels
     */
    DriveMotors(pros::MotorGroup* left, pros::MotorGroup* right, double wheelDiameter);

    /**
     * @brief Set motor voltages for both sides of the drivetrain
    */
    void setVoltage(double leftMv, double rightMv) const;

    /**
     * @brief Aracade style driver controls built into yogurtlib, for use in the driver control period of a match 
     * 
     * @param power the forwards and backwards power
     * @param turn the turning power
     * @param useExpo whether to use exponential scaling for the drive control, true = on
     * @param expo the controller for exponential driving, 1.0 = linear driving, closer to 0 will make the robot accelerate to higher speeds with less stick diflection, while values above 1.0 will make deflection near the center softer
     * @param bias how much turning or driving should take priority if they excceed full speed, ranges between 0 - 1 where 0 is all driving, 1 is all turning
     * 
     * ### Example code:
     * 
     * ```cpp
     * void opcontrol() {
     *     while(true){
     *         drivetrain.motors.arcade(master.get_analog(analog_left_Y), 
     *                                  master.get_analog(analog_right_X),
     *                                  3.0);
     *     }
     * }
     * ```
     */
    void arcade(double power, double turn, bool useExpo = false, double expo = 2.0, double bias = 0.5) const;

    /**
     * @brief Tank style driver controls built into yogurtlib, for use in the driver control period of a match 
     * 
     * @param leftPower the power that should be given to the left motors
     * @param rightPower the power that should be given to the right motors
     * @param useExpo whether to use exponential scaling for the drive control
     * @param expo the controller for exponential driving, 1.0 = linear driving, closer to 0 will make the robot accelerate to higher speeds with less stick diflection, while values above 1.0 will make deflection near the center softer
     *   
     * ### Example code:
     * 
     * ```cpp
     * void opcontrol() {
     *     while(true){
     *         drivetrain.motors.tank(master.get_analog(analog_left_Y), 
     *                                master.get_analog(analog_right_Y),
     *                                3.5);
     *     }
     * }
     * ```
     */
    void tank(double leftPower, double rightPower, bool useExpo = false, double expo = 2.0) const;

    /**
     * @brief Sets the brakemode of the motors, choose between:
     *
     * - Break
     *
     * - Hold
     *
     * - Coast
     *
     * ### Example code:
     *
     * ```cpp
     * void initialize(){
     *    drivetrain.motors.setBrakeMode(yogurt::BrakeMode::Coast);
     * }
     */
    void setBrakeMode(BrakeMode mode) const;

    
    /**
     * @brief Stops the drivetrains movement
     */
    void stop(BrakeMode mode = BrakeMode::Brake) const;

    /**
     * @brief Returns the last voltage targets for each side of the drivetrain, left first, then right
    */
    std::pair<double, double> getLastVolt() const;

    /**
     * @brief Returns the wheel diameter of the drivetrain
    */
    double getWheelDiameter() const;

private:
    double wheelDiameter;   // inches
    mutable double lastL = 0.0, lastR = 0.0; //Records last voltage for both sides
    double expoCurve(double value, double n) const;
};

} // namespace yogurt
