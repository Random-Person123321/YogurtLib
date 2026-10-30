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

    DriveMotors(pros::MotorGroup* left, pros::MotorGroup* right, double wheelDiameter);

    /**
     * @brief Set motor voltages for both sides of the drivetrain
    */
    void setVoltage(double leftMv, double rightMv) const;

    /**
     * @brief Aracade style driver controls built into yogurtlib, for use in the driver control period of a match 
     * 
     * ### Example code:
     * 
     * ```cpp
     * void opcontrol() {
     *     while(true){
     *         drivetrain.motors.arcade(master.get_analog(analog_left_Y), 
     *                                  master.get_analog(analog_right_X);
     *     }
     * }
     * ```
     */
    void arcade(double power, double turn) const;

    /**
     * @brief Tank style driver controls built into yogurtlib, for use in the driver control period of a match 
     * 
     * ### Example code:
     * 
     * ```cpp
     * void opcontrol() {
     *     while(true){
     *         drivetrain.motors.tank(master.get_analog(analog_left_Y), 
     *                                master.get_analog(analog_right_Y);
     *     }
     * }
     * ```
     */

    void tank(double leftPower, double rightPower) const;

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
     *    drivetrain.motors.setBrakeMode(yogurt:BrakeMode::Coast);
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
};

} // namespace yogurt
