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

    poseMutex.take();
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
    poseMutex.give();

    lastVertTicks = vNow;
    lastHorizTicks = hNow;
    lastHeadingDeg = headDeg;
}

// double IMU::fuseIMUs(double time) {
//     double maxDegMs = maxTurnDelta / 1000.0 * time;
//     double median = 0.0;
//     double sorted = 0.0;
//     int goodImus = 0;
//     std::vector<std::pair<double, int>> sortedImu;

//     for(int i = 0; i < sensors.size(); i++){
//         imuData[i].skip = false; //Reset so no IMU's are skipped
//         imuData[i].badRead = false;
//     }

//     if (sensors.size() >= 2){
//         for (int i = 0; i < sensors.size(); i++) {
//             double raw = sensors[i]->get_rotation();
//             double reading = raw * imuData[i].scale;
//             imuData[i].badRead = Imubad(reading);
//             if (imuData[i].badRead) {imuData[i].skip = true; imuData[i].rejects += 1; continue;}
//             if (!imuData[i].badRead) {imuData[i].delta = (imuData[i].last - raw) * imuData[i].scale; imuData[i].last = raw; }
//             //Skip logic based on max speed (D/s the IMU can read befor being rejected)
//             if(abs(imuData[i].delta) >= maxDegMs) {imuData[i].rejects += 1; imuData[i].skip = true; }
//             if (imuData[i].skip || imuData[i].dead || imuData[i].benched) continue;
//             sortedImu.push_back({imuData[i].delta, i});
//         }
//         std::sort(sortedImu.begin(), sortedImu.end());
//         int asize = sortedImu.size();
//         if (asize == 0) return lastFused = 0.0;
//         if (asize % 2 == 1) median = sortedImu[asize / 2].first;
//         else median = (sortedImu[asize / 2 - 1].first + sortedImu[asize / 2].first) / 2.0;
//         double tolerance = inertialRejectTolerance + (inertialTurnLeniency * fabs(median));
//         for (int k = 0; k < asize; k++) {
//             double d = sortedImu[k].first;    // the delta
//             int idx  = sortedImu[k].second;   // which IMU it came from
//             if (std::fabs(median - d) < tolerance) {
//                 sorted += d;
//                 goodImus += 1;
//             } else {
//                 imuData[idx].rejects += 1;
//             }
//         }
//         lastFused = (goodImus > 0) ? sorted / goodImus : median;

//         if (asize >= 3){
//             //Go into benching logic
//             for(int i = 0; i < sensors.size(); i++){
//                 if(imuData[i].dead || imuData[i].skip) continue;
//                 if(imuData[i].benched){
//                     imuData[i].drift = (imuData[i].drift + (imuData[i].delta - lastFused)) * benchedImuDecay;
//                     if (imuData[i].drift < inertialRejectTolerance) {imuData[i].goodstreak += 1;}
//                     else {imuData[i].goodstreak = 0;}
//                     if (imuData[i].goodstreak >= benchedImuGoodRuns) {imuData[i].goodstreak = 0; imuData[i].benched = false; imuData[i].badstreak = 0;}
//                 } 
//                 else if (!imuData[i].benched){
//                     imuData[i].drift = (imuData[i].drift + (imuData[i].delta - lastFused)) * benchedImuDecay;
//                     if(imuData[i].drift >= inertialRejectTolerance) {imuData[i].badstreak += 1;}
//                     else {if(imuData[i].badstreak > 0) {imuData[i].badstreak -=1; } else {imuData[i].goodstreak += 1; }}
//                     if (imuData[i].badstreak >= benchedImuBadRuns) {imuData[i].benched = true; imuData[i].goodstreak = 0; imuData[i].badstreak = 0;}
//                 }
//             }
//         }

//         return lastFused; //Return the fused and filtered imu data
//     }
//     else if (sensors.size() == 1){
//         double raw = sensors[0]->get_rotation();
//         if (Imubad(raw)){
//             return 0; //Return 0 if the IMU reads inf or nan - assuming no heading change
//         } else {
//             double delta = imuData[0].last - raw;
//             imuData[0].last = raw;
//             lastFused = delta;
//             return lastFused; //Return the difference in rotation - DEGREES
//         }
//     } else {
//         return 0; //If 0 imu's or other errors
//     }
// }

void Drivetrain::distanceReset(DistanceSensors& distance, double knownFeaturePos, int resets, double angleOffset) { //The angle offset the sensor is mounted at (Pointint up = (+) vice verca)
    double offset = distance.isXaxis ? distance.xOffset : distance.yOffset;
    auto readInches = [&]() -> double {
    int mm = distance.sensor->get();
    if (mm == PROS_ERR) return NAN;
    return mm / 25.4;
    };

  auto reset = [&]() {
    angleOffset = std::abs(angleOffset);
    angleOffset = deg2rad(angleOffset);

    double a = readInches(); pros::delay(34);
    double b = readInches(); pros::delay(34);
    double c = readInches();

    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) return;

    double reading = (a + b + c) / 3.0;
    if (reading < 1.5 || reading > 78.0) return;
    // Adjust reading based on the angle offset the sensor is mounted at
    reading = std::cos(angleOffset) * reading;

    if (std::fabs(a - b) > 2.0 || std::fabs(b - c) > 2.0 || std::fabs(a - c) > 2.0) return;

    double sign = distance.sensorDirPositive ? -1.0 : 1.0;
    double corrected = knownFeaturePos + sign * (reading + offset);

    constexpr double alpha = 0.6; //0.35
    if (distance.isXaxis) pose.x += (corrected - pose.x) * alpha;
    else pose.y += (corrected - pose.y) * alpha;
  };
  poseMutex.take();
  for (int i = 1; i <= resets; i++) {
    reset();
    pros::delay(50);
  }
  poseMutex.give();
}

bool IMU::Imubad (double value) {
    return std::isnan(value) || std::isinf(value);
}

//READ

void IMU::readSensors(double timeMs) {
    const double maxStep = maxTurnDelta * timeMs / 1000.0;
    for (int i = 0; i < sensors.size(); i++) imuData[i].valid = checkIMU(i, maxStep);
}

bool IMU::checkIMU(int i, double maxStep) {
    auto& d = imuData[i];
    if (sensors[i]->is_calibrating()) {
        d.resync = true;
        if (sensors.size() >= 3) { d.benched = true, d.goodstreak = 0; }
        return false;
    }
    double raw = sensors[i]->get_rotation();
    if (Imubad(raw)) { d.rejects++; d.resync = true; return false; }
    if (d.resync) { d.last = raw; d.resync = false; return false; }
    d.delta = (d.last - raw) * d.scale - d.bias;
    d.last = raw;
    if (std::fabs(d.delta) > maxStep) { d.rejects++; return false; };
    return true;
}

// Combine
bool IMU::fillVals() {
    vals.clear();
    for (auto &d : imuData) if (d.valid && !d.benched) vals.push_back(d.delta);
    if (vals.empty()) for (auto&d : imuData) if (d.valid) vals.push_back(d.delta);
    return !vals.empty();
}

double IMU::median() {
    std::sort(vals.begin(), vals.end());
    int n = vals.size();
    if (n % 2 == 0) { // If even, take the average of the two in the middle for median
        return (vals[n / 2 - 1] + vals[n / 2]) / 2.0;
    }
    else return vals[n / 2]; // Odd, return the middle value
}

double IMU::rejectionTolerance(double median) const {
    return inertialRejectTolerance + inertialTurnLeniency * std::fabs(median);
}
