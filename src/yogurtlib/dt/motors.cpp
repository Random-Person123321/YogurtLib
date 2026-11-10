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

void DriveMotors::arcade(double power, double turn, bool useExpo, double expo, double bias) const {
    if(!useExpo) expo = 1.0; // If users elect not to use expo drive we set it to linear
    // Get the abs and clamp between good values
    expo = std::fabs(expo);
    bias = std::fabs(bias);
    expo = std::clamp(expo, 0.1, 146.0);
    bias = std::clamp(bias, 0.0, 1.0);

    power = expoCurve(std::clamp(power, -127.0, 127.0), expo);
    turn = expoCurve(std::clamp(turn, -127.0, 127.0), expo);

    double leftPower = power + turn;
    double rightPower = power - turn;

    double maxV = std::max(std::fabs(leftPower), std::fabs(rightPower));

    if (maxV > 127.0) {
        double limitTurn = std::copysign(127.0 - std::fabs(power), turn);
        double lDrive = power + limitTurn;
        double rDrive = power - limitTurn;

        double limitPower = std::copysign(127.0 - std::fabs(turn), power);
        double lTurn = limitPower + turn;
        double rTurn = limitPower - turn;

        leftPower  = lDrive * (1.0 - bias) + lTurn * bias;
        rightPower = rDrive * (1.0 - bias) + rTurn * bias;
    }

    leftPower = leftPower * 12000.0 / 127.0;
    rightPower = rightPower * 12000.0 / 127.0;

    setVoltage(leftPower, rightPower);
}

void DriveMotors::tank(double leftPower, double rightPower, bool useExpo, double expo) const {
    if(!useExpo) expo = 1.0; // If users elect not to use expo drive we set it to linear
    expo = std::fabs(expo);
    expo = std::clamp(expo, 0.1, 146.0);


    leftPower = expoCurve(std::clamp(leftPower, -127.0, 127.0), expo);
    rightPower = expoCurve(std::clamp(rightPower, -127.0, 127.0), expo);

    leftPower = std::clamp(leftPower, -127.0, 127.0);
    rightPower = std::clamp(rightPower, -127.0, 127.0);

    leftPower = leftPower * 12000.0 / 127.0;
    rightPower = rightPower * 12000.0 / 127.0;

    setVoltage(leftPower, rightPower);
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

double DriveMotors::expoCurve(double value, double n) const {
    if (value == 0.0) return 0.0; // If the value is zero return a zero
    double numi = value * std::pow(std::fabs(value), n - 1.0);
    double denom = std::pow(127.0, n - 1.0);
    return numi / denom;
}