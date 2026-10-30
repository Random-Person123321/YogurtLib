#include "yogurtlib/control.hpp"
#include <algorithm>
#include <cmath>
using namespace yogurt;

PID::PID(double p, double i, double d, double maxInt)
         : kP(p), kI(i), kD(d), maxIntegral(maxInt) {}


void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
    first = true;
}

double PID::calculateError(double error, double dt) {
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

double PID::calculate(double target, double current, double dt) {
    return calculateError(target - current, dt);
}

void Settle::reset() { lastErr = 1e9; goodMs = 0; }

bool Settle::update(double err, double dt) {
    double vel = (dt > 0) ? std::fabs((err - lastErr) / dt) : 1e9;
    lastErr = err;

    if (std::fabs(err) < errTol && vel < velTol) {
        goodMs += (int)std::round(dt * 1000.0);
    } 
    else goodMs = 0;

    return goodMs >= settleMs;
}