#include "yogurtlib/dt/dt.hpp"
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
