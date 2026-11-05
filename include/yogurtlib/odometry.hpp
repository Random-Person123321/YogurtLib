#pragma once
#include "main.h"
#include <vector>

namespace yogurt {

//IMU data struct
struct ImuDataVars {
    double last = 0.0;
    double delta = 0.0;
    double drift = 0.0;
    double scale = 1.0;
    double bias = 0.0;
    bool valid = false;
    bool benched = false;
    bool resync = false;
    // bool skip = false;
    // bool dead = false;
    // bool badRead = false;
    int rejects = 0;
    int goodstreak = 0, badstreak = 0;
};


class IMU {
    
public:

   //IMU fusion VARIABLES:
    double inertialRejectTolerance = 0.3; //Degrees IMU's are allowed to drift before being excluded from averaging
    double inertialTurnLeniency = 0.10; //Percent that tolerance scales based on fast turns
    double maxTurnDelta = 720.0; //Maximum speed a IMU can measure, if an IMU reads above this it is ignored, in D/s

    double maxDriftRate = 0.5; // deg/s, if the IMU drifts faster than this than it is no longer considered for heading
    double driftWindow = 2.0; // seconds, this is for how long during the run the drift check considers the IMU for

    int benchedImuGoodRuns = 100; //How long of a good streak any benched IMU's must have before being allowed to rejoin the averaging
    int benchedImuBadRuns = 25; //How long of a bad streak any IMU's require to become benched
    double biasLearnRate = 0.01;
    double stillMaxDelta = 0.05;

    std::vector<pros::Imu*> sensors;
    std::vector<yogurt::ImuDataVars> imuData;

    IMU(std::vector<pros::Imu*> sensors) : sensors(sensors) {imuData.resize(sensors.size()); }

    bool calibrate(int timeoutMs = 3000) {
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

    void rebase() {
        for (int i = 0; i < sensors.size(); i++) {
            double raw = sensors[i]->get_rotation();
            if (!Imubad(raw)) imuData[i].last = raw;
            else imuData[i].resync = true;
        }
    }

    bool anyCalibrating() {
        for (auto&s : sensors) {
            if (s->is_calibrating()) return true;
        }
        return false;
    }

    double lastFused = 0.0;
    // double totalHeading = 0.0; //This belongs in the drivetrain class
    /**
     * @brief !!!!! UNTESTED !!!!!
     *
     * The funtion that (should be able) to take any number of IMU's and filter them, than return the delta
    */
    double fuseIMUs(double time = 10.0, bool still = false);

    bool Imubad (double value);

private:
    std::vector<double> vals;
    // Read Sensors
    void readSensors(double timeMs);
    bool checkIMU(int i, double maxStep);
    // Combine
    bool fillVals();
    double median();
    double rejectionTolerance(double median) const;

    double acceptIMU(const ImuDataVars& d, double median, double tol) const;
    double averageAccepted(double median, double tol) const;
    // Health
    void updateHealth(double median, double tol, double timeMs);
    void markGood(ImuDataVars& d);
    void markBad(ImuDataVars& d);
    // Bias
    void learnBias();
};
}; //namespace yogurt