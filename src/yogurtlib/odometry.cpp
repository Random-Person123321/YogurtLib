#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

double Drivetrain::readHeadingDeg() const {
    double sumSin = 0.0, sumCos = 0.0;
    int n = 0;
    for (auto* imu : odom.imus.sensors) {
        double raw = imu->get_heading();
        if (std::isnan(raw) || std::isinf(raw)) continue;
        double mathDeg = wrapDeg(90.0 - raw);
        sumSin += std::sin(deg2rad(mathDeg));
        sumCos += std::cos(deg2rad(mathDeg));
        ++n;
    }
    if (n == 0) return pose.theta;
    return wrapDeg(rad2deg(std::atan2(sumSin, sumCos)));
}

void Drivetrain::odomLoop() {
    const int stepMs = 10;
    std::uint32_t prevTime = pros::millis();
    while (true) {
        updateOdometry(stepMs / 1000.0);
        pros::Task::delay_until(&prevTime, stepMs);
    }
}

void Drivetrain::updateOdometry(double /*dt*/) {
    const double V_IN_PER_TICK = inchesPerTick(odom.verticalDiameter);
    const double H_IN_PER_TICK = inchesPerTick(odom.horizontalDiameter);

    double vNow = odom.vertical ? odom.vertical->get_position() : lastVertTicks;
    double hNow = odom.horizontal ? odom.horizontal->get_position() : lastHorizTicks;
    if (std::isnan(vNow) || std::isinf(vNow)) vNow = lastVertTicks;
    if (std::isnan(hNow) || std::isinf(hNow)) hNow = lastHorizTicks;

    double headDeg = odom.imus.sensors.empty() ? pose.theta : readHeadingDeg();
    double dThetaDeg = angleDiffDeg(headDeg, lastHeadingDeg);
    double dTheta = deg2rad(dThetaDeg);

    double dV = (vNow - lastVertTicks) * V_IN_PER_TICK;
    double dH = (hNow - lastHorizTicks) * H_IN_PER_TICK;

    double forward = dV - (odom.verticalOffset * dTheta);
    double left = dH - (odom.horizontalOffset * dTheta);

    double mid = deg2rad(lastHeadingDeg) + dTheta * 0.5;
    double cosT = std::cos(mid);
    double sinT = std::sin(mid);

    pose.x += forward * cosT - left * sinT;
    pose.y += forward * sinT + left * cosT;
    pose.theta = headDeg;

    lastVertTicks = vNow;
    lastHorizTicks = hNow;
    lastHeadingDeg = headDeg;
}