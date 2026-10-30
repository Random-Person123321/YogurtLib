#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/dt/motors.hpp"
using namespace yogurt;

double Drivetrain::limitOutput(double v, double& prev, double minV, double maxV, double slewRate) const {
    v = std::clamp(v, minV, maxV);
    v = slew(v, prev, slewRate);
    prev = v;
    return v;
}

bool Drivetrain::chainExit (double error, int dir, double minV, double earlyExitRange) {
    if (minV <= 0.0) return false;
    return error * dir < 0.0 || fabs(error) < earlyExitRange;
}

double Drivetrain::inchesPerTick(double wheelDiamIn) {
    return (wheelDiamIn * M_PI) / 36000.0;
}

pros::motor_brake_mode_e_t yogurt::toProsBrakeMode(BrakeMode mode) {
    switch (mode) {
        case BrakeMode::Coast: return MOTOR_BRAKE_COAST;
        case BrakeMode::Hold:  return MOTOR_BRAKE_HOLD;
        case BrakeMode::Brake: default :
                               return MOTOR_BRAKE_BRAKE;
    }
}

BrakeMode yogurt::toBrakeMode(pros::MotorBrake mode) {
    switch (mode) {
        case pros::MotorBrake::coast : return BrakeMode::Coast;
        case pros::MotorBrake::hold :  return BrakeMode::Hold;
        case pros::MotorBrake::brake : default: return BrakeMode::Brake;
    }
}