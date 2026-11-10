#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/misc.hpp"
using namespace yogurt;

// double Drivetrain::readHeadingDeg() const {
//     double sumSin = 0.0, sumCos = 0.0;
//     int n = 0;
//     for (auto* imu : odom.imus.sensors) {
//         double raw = imu->get_heading();
//         if (std::isnan(raw) || std::isinf(raw)) continue;
//         double mathDeg = wrapDeg(90.0 - raw);
//         sumSin += std::sin(deg2rad(mathDeg));
//         sumCos += std::cos(deg2rad(mathDeg));
//         ++n;
//     }
//     if (n == 0) return pose.theta;
//     return wrapDeg(rad2deg(std::atan2(sumSin, sumCos)));
// }

void Drivetrain::odomLoop() {
    const int stepMs = 10;
    std::uint32_t prevTime = pros::millis();
    int logCount = 0;
    while (true) {
        updateOdometry(stepMs / 1000.0);
        if (++logCount % 10 == 0) {
        printf("Still:%d",lastStill);
        for (int i = 0; i < odom.imus.count(); i++) {
            printf(",Heading: %.1f", odom.imus.getHeading(i));
            printf(",Drift: %.3f", odom.imus.drift(i));
            printf(",Rejects: %d", odom.imus.imuData[i].rejects);

        }
        printf("\n"); 
}
        pros::Task::delay_until(&prevTime, stepMs);
    }
}

void Drivetrain::updateOdometry(double dt) {
    const double V_IN_PER_TICK = inchesPerTick(odom.verticalDiameter);
    const double H_IN_PER_TICK = inchesPerTick(odom.horizontalDiameter);

    double vNow = odom.vertical ? odom.vertical->get_position() : lastVertTicks;
    double hNow = odom.horizontal ? odom.horizontal->get_position() : lastHorizTicks;
    if (std::isnan(vNow) || std::isinf(vNow)) vNow = lastVertTicks;
    if (std::isnan(hNow) || std::isinf(hNow)) hNow = lastHorizTicks;

    poseMutex.take();

    double dV = (vNow - lastVertTicks) * V_IN_PER_TICK;
    double dH = (hNow - lastHorizTicks) * H_IN_PER_TICK;

    bool still = isStill(dV, dH, dt);
    lastStill = still;
    double dThetaDeg = odom.imus.fuseIMUs(dt * 1000.0, still);
    double dTheta = deg2rad(dTheta);

    double forward = dV - (odom.verticalOffset * dTheta);
    double left = dH - (odom.horizontalOffset * dTheta);

    double mid = deg2rad(pose.theta) + dTheta * 0.5; // Last cycle's reading averaged with the current cycle's delta
    double cosT = std::cos(mid);
    double sinT = std::sin(mid);

    pose.x += forward * cosT - left * sinT;
    pose.y += forward * sinT + left * cosT;
    pose.theta += dThetaDeg; // Unwrapped internally
    poseMutex.give();

    lastVertTicks = vNow;
    lastHorizTicks = hNow;
}


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
    poseMutex.take();
    if (distance.isXaxis) pose.x += (corrected - pose.x) * alpha;
    else pose.y += (corrected - pose.y) * alpha;
    poseMutex.give();
  };

  for (int i = 1; i <= resets; i++) {
    reset();
    pros::delay(50);
  }
}

std::vector<double> Drivetrain::measureImuScale(bool CW, int turns, int volts, int resetSecs, int timeOut) {
    double dir = CW ? -1.0 : 1.0;
    auto& imus= odom.imus;
    std::vector<double> start(imus.sensors.size());
    for (size_t i = 0; i < start.size(); i++) start[i] = imus.sensors[i]->get_rotation();

    Pose p = getPose();
    double startHeading = p.theta;
    motors.setVoltage(-dir * volts, dir * volts);
    int elapsed = 0;

    while (std::fabs(p.theta - startHeading) < turns * 360.0 - 10.0 && elapsed < timeOut) {
        p = getPose();
        pros::delay(10);
        elapsed += 10;
    }

    motors.stop(BrakeMode::Coast);

    printf("Square the robot... %d seconds \n", resetSecs);
    pros::delay(resetSecs * 1000);
    
    std::vector<double> scales(start.size(), 1.0);
    for(size_t i = 0; i < start.size(); i++) {
        double measured = std::fabs(imus.sensors[i]->get_rotation() - start[i]);
        if(std::isinf(measured) || measured < 180.0) continue;
        double trueTurns = std::round(measured / 360.0) * 360.0;
        scales[i] = trueTurns / measured;
    }

    return scales;
}

void IMU::setScale(int i, int scale) {
    if (i < 0 || i >= imuData.size() || scale <= 0.0) return;
    imuData[i].scale = scale;
}

int IMU::acceptedIMUs() const {
    int cnt = 0;
    for (auto &d : imuData) if (d.valid && !d.benched) cnt++;
    return cnt;
}

bool IMU::isBenched(int i) const {
    if (i < 0 || i >= imuData.size()) return false;
    return imuData[i].benched;
}

bool IMU::isValid(int i) const {
    if (i < 0 || i >= imuData.size()) return false;
    return imuData[i].valid;
}

double IMU::drift(int i) const {
    if (i < 0 || i >= imuData.size()) return 0.0;
    return imuData[i].drift;
}

// Temporary for now
double IMU::getHeading(int i) const {
    if (i < 0 || i >= imuData.size()) return 0.0;
    return wrapDeg(imuData[i].last);
}

int IMU::count() const {
    return sensors.size();
}

double IMU::fuseIMUs(double timeMs, bool still) {
    if (sensors.empty()) return lastFused = 0.0;
    
    readSensors(timeMs);
    if (!fillVals()) return lastFused = 0.0;

    double median = IMU::median();
    double tol = rejectionTolerance(median);
    lastFused = averageAccepted(median, tol);
    if (sensors.size() >= 3) updateHealth(median, tol, timeMs);
    if (still) learnBias();
    
    return lastFused;
}

bool IMU::calibrate(int timeoutMs) {
    for(auto &s : sensors) s->reset(false);
    pros::delay(50);
    for (int t = 50; t < timeoutMs; t += 10) pros::delay(10);

    bool allReady = true;
    for (int i = 0; i < sensors.size(); i++) {
        double scale = imuData[i].scale;
        imuData[i] = ImuDataVars{}; // Clears old values for new ones
        imuData[i].scale = scale; //   Reinputs old scale
        imuData[i].resync = true; // Figure out how to do
        if (sensors[i]->is_calibrating()) allReady = false;
        sensors[i]->set_data_rate(5);
    }
    return allReady;
}

void IMU::rebase() {
    for (int i = 0; i < sensors.size(); i++) {
        double raw = sensors[i]->get_rotation();
        if (!Imubad(raw)) imuData[i].last = raw;
        else imuData[i].resync = true;
    }
}

bool IMU::anyCalibrating() {
    for (auto&s : sensors) {
        if (s->is_calibrating()) return true;
    }
    return false;
}

bool IMU::Imubad (double value) {
    return std::isnan(value) || std::isinf(value);
}

bool Drivetrain::isStill(double dV, double dH, double dt) {
    auto [l, r] = motors.getLastVolt();
    bool quiet = l == 0.0 && r == 0.0
                 && std::fabs(dV) < stillWheelTol
                 && std::fabs(dH) < stillWheelTol;
    if (!quiet) stillMs = 0;
    else if (stillMs < stillTime) stillMs += (int) (dt * 1000.0);
    return stillMs >= stillTime;
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
    // Using last - current to convert compass heading into unit circle convention(CCW positive deg)
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

bool IMU::acceptIMU(const ImuDataVars& d, double median, double tol) const {
    return std::fabs(d.delta - median) <= tol;
}

double IMU::averageAccepted(double median, double tol) const {
    double sum = 0.0; int cnt = 0;
    for (auto &d : imuData) {
        if (d.valid && !d.benched && acceptIMU(d, median, tol)) {
            sum += d.delta; cnt++;
        }
    }
    return cnt ? sum / cnt : median;
}

// Health
void IMU::updateHealth(double median, double tol, double timeMs) {
    const double dt = timeMs / 1000.0;
    const double decay = 1.0 - dt / driftWindow;
    const double driftLimit = maxDriftRate * driftWindow;
    for (auto &d : imuData) {
        if (!d.valid) continue;
        // Decays the drift over time and add the new drift from the known blended value
        d.drift = d.drift * decay + d.delta - lastFused;
        bool good = acceptIMU(d, median, tol) && std::fabs(d.drift) <= driftLimit;
        if (good) markGood(d);
        else markBad(d);
    }
}

void IMU::markGood(ImuDataVars& d) {
    if (d.badstreak > 0) d.badstreak--;
    if (d.benched && ++d.goodstreak >= benchedImuGoodRuns) {
        d.benched = false;
        d.goodstreak = 0;
    }
}

void IMU::markBad(ImuDataVars& d) {
    d.goodstreak = 0;
    d.rejects++;
    if(!d.benched && ++d.badstreak >= benchedImuBadRuns){
        d.benched = true;
        d.badstreak = 0;
    }
}

// Bias
void IMU::learnBias() {
    for (auto &d: imuData) {
        // If the IMU is good and the delta is small enough(meaning we are still)
        // Update the bias overtime according to a learn speed percentage
        if (d.valid && std::fabs(d.delta) < stillMaxDelta) d.bias += biasLearnRate * d.delta;
    }
}