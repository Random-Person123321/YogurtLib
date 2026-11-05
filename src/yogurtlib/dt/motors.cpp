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

void DriveMotors::arcade(double power, double turn, double expo, double bias) const {
    // Get the abs and clamp between good values
    expo = std::fabs(expo);
    bias = std::fabs(bias);
    expo = std::clamp(expo, 0.2, 127);
    bias = std::clamp(bias, 0, 1);

    power = expoCurve(power, expo);
    turn = expoCurve(turn, expo);

    double left = power + turn;
    double right = power - turn;

    double max = std::max(std::fabs(left), std::fabs(right));

    if (max > 127) {
        double cap = 127 + (bias - 1.0) * std::fabs(turn);

        double scale = cap / (max - std::fabs(turn) * bias);
        
        left = left * scale;
        right = right * scale;
    }

    left->move(left);
    right->move(right);
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

double expoCurve(double value, double n) {
    double numi = value * std::fabs(std::pow(std::fabs(value), n - 1));
    double denom = std::fabs(std::pow(127, n - 1));
    return numi / denom;
}