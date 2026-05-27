#pragma once
#include "main.h"

namespace Yogurt {
// ---- PID (uses dt, proven to work) ----
class PID {
private:
    double kP;
    double kI;
    double kD;

    double maxIntegral;   // clamp on integral term magnitude (0 disables clamp)
    double integral = 0.0;
    double prevError = 0.0;
    bool first = true;

public:
    PID(double p, double i, double d, double maxInt)
        : kP(p), kI(i), kD(d), maxIntegral(maxInt) {}

    void reset() {
        integral = 0.0;
        prevError = 0.0;
        first = true;
    }

    // Best method: feed error directly
    double calculateError(double error, double dt) {
        if (dt <= 0) dt = 0.01;

        integral += error * dt;
        if (maxIntegral != 0) {
            integral = std::clamp(integral, -maxIntegral, maxIntegral);
    }

    double derivative = 0.0;
        if (!first) derivative = (error - prevError) / dt;
        first = false;

        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    // Standard method: target/current
    double calculate(double target, double current, double dt) {
        return calculateError(target - current, dt);
    }

    // Backwards compatible (assumes 10ms loop)
    double calculate(double target, double current) {
        return calculate(target, current, 0.01);
    }
};
}//Yogurt namespace