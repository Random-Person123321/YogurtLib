#pragma once
#include "api.h"

namespace yogurt{
    struct ControllerSettings {
        double kp, ki, kd, maxIntegral;

        double errTol, velTol;
        int settleMs;

        ControllerSettings(double p, double i, double d, double maxInt,
                           double errTol, double velTol)
        : kp(p), ki(i), kd(d), maxIntegral(maxInt) {}
    };

    class Drivetrain {
        pros::MotorGroup* left_mg;
        pros::MotorGroup* right_mg;

    };
}//yogurt namespace