#include "yogurtlib/dt/motors.hpp"
#include <algorithm>
using namespace yogurt;


DriveMotors::DriveMotors(pros::MotorGroup* left, pros::MotorGroup* right, double wheelDiameter) :
                         left(left), right(right), wheelDiameter(wheelDiameter) {}
              

void DriveMotors::setVoltage(double leftMv, double rightMv) const {
    left->move_voltage((std::int32_t)leftMv);
    right->move_voltage((std::int32_t)rightMv);
    lastL = leftMv;
    lastR = rightMv;
}

void DriveMotors::arcade(double power, double turn) const {
    tank(power + turn, power - turn);
}

void DriveMotors::tank(double leftPower, double rightPower) const {
    left->move(std::clamp(leftPower, -127.0, 127.0));
    right->move(std::clamp(rightPower, -127.0, 127.0));
}

void DriveMotors::setBrakeMode(BrakeMode mode) const {
    left->set_brake_mode_all(toProsBrakeMode(mode));
    right->set_brake_mode_all(toProsBrakeMode(mode));
}

void DriveMotors::stop(BrakeMode mode) const {
    setBrakeMode(mode);
    left->move_voltage(0);
    right->move_voltage(0);
    lastL = 0.0; lastR = 0.0;
}

std::pair<double, double> DriveMotors::getLastVolt() const {return std::make_pair(lastL, lastR); }

double DriveMotors::getWheelDiameter() const { return wheelDiameter; }