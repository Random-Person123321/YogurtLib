#pragma once
#include "main.h"

namespace yogurt {

enum class BrakeMode {
    Coast,
    Brake,
    Hold
};

inline pros::motor_brake_mode_e_t toProsBrakeMode(BrakeMode mode) {
    switch (mode) {
        case BrakeMode::Coast: return MOTOR_BRAKE_COAST;
        case BrakeMode::Hold:  return MOTOR_BRAKE_HOLD;
        case BrakeMode::Brake:
        default:               return MOTOR_BRAKE_BRAKE;
    }
}

inline BrakeMode toBrakeMode(pros::MotorBrake mode) {
    switch (mode) {
        case pros::MotorBrake::coast : return BrakeMode::Coast;
        case pros::MotorBrake::hold :  return BrakeMode::Hold;
        case pros::MotorBrake::brake : default: return BrakeMode::Brake;
    }
}

// Raw drivetrain specs
class DriveMotors {
public:
    pros::MotorGroup* left;
    pros::MotorGroup* right;

    DriveMotors(pros::MotorGroup* left, pros::MotorGroup* right,
                double wheelDiameter)
        : left(left), right(right),
          wheelDiameter(wheelDiameter) {}

    // Direct motor voltage, in mV [-12000, 12000].
    void setVoltage(double leftMv, double rightMv) const {
        left->move_voltage((std::int32_t)leftMv);
        right->move_voltage((std::int32_t)rightMv);
        lastL = leftMv;
        lastR = rightMv;
    }

    // Arcade drive. power/turn in motor "move" units [-127, 127].
    void arcade(double power, double turn) const {
        tank(power + turn, power - turn);
    }

    // Tank drive. left/right power in motor "move" units [-127, 127].
    void tank(double leftPower, double rightPower) const {
        left->move(std::clamp(leftPower, -127.0, 127.0));
        right->move(std::clamp(rightPower, -127.0, 127.0));
    }

    void setBrakeMode(BrakeMode mode) const {
        left->set_brake_mode_all(toProsBrakeMode(mode));
        right->set_brake_mode_all(toProsBrakeMode(mode));
    }

    void stop(BrakeMode mode = BrakeMode::Brake) const {
        setBrakeMode(mode);
        left->move_voltage(0);
        right->move_voltage(0);
        lastL = 0.0; lastR = 0.0;
    }

    std::pair<double, double> getLastVolt() const {return std::make_pair(lastL, lastR); }
    double getWheelDiameter() const { return wheelDiameter; }

private:
    double wheelDiameter;   // inches
    mutable double lastL = 0.0, lastR = 0.0; //Records last voltage for both sides
};

} // namespace yogurt
