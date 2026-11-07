#pragma once
#include "main.h"
#include <vector>

namespace yogurt {

class Drivetrain;

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
    int rejects = 0;
    int goodstreak = 0, badstreak = 0;
};


class IMU {
    
public:

   //IMU fusion VARIABLES:
    double inertialRejectTolerance = 0.3; //Degrees IMU's are allowed to differ from the median before being excluded from averaging
    double inertialTurnLeniency = 0.5; //Percent that tolerance scales based on fast turns
    double maxTurnDelta = 2000.0; //Maximum speed a IMU can measure, if an IMU reads above this it is ignored, in deg/s

    double maxDriftRate = 0.25; // deg/s, if the IMU drifts faster than this than it is no longer considered for heading
    double driftWindow = 2.0; // seconds, this is for how long during the run the drift check considers the IMU for

    int benchedImuGoodRuns = 100; //How long of a good streak any benched IMU's must have before being allowed to rejoin the averaging
    int benchedImuBadRuns = 25; //How long of a bad streak any IMU's require to become benched
    double biasLearnRate = 0.01;
    double stillMaxDelta = 0.05;

    IMU(std::vector<pros::Imu*> sensors) : sensors(sensors) {imuData.resize(sensors.size()); }

    void setScale(int i, int scale); // Sets the scale for IMU i. (Tune with tuneIMUScale function)
    
    int acceptedIMUs() const; // IMUs that are giving both valid readings and currently not benched(agrees with rest)

    bool isBenched(int i) const;

    bool isValid(int i) const;

    double drift(int i) const;

    double getHeading(int i) const;

    int count() const;
private:
    friend class Drivetrain;

    double fuseIMUs(double timeMs = 10.0, bool still = false);

    bool calibrate(int timeoutMs = 3000);

    void rebase();

    bool anyCalibrating();

    bool Imubad (double value);

    std::vector<pros::Imu*> sensors;
    double lastFused = 0.0;
    std::vector<double> vals;
    std::vector<yogurt::ImuDataVars> imuData;
    // Above are the functions, that Drivetrain would call as the class' friend
    // While below are the actual private parts that IMU handles internally

    // Read Sensors
    void readSensors(double timeMs);
    bool checkIMU(int i, double maxStep);
    // Combine
    bool fillVals();
    double median();
    double rejectionTolerance(double median) const;
    bool acceptIMU(const ImuDataVars& d, double median, double tol) const;
    double averageAccepted(double median, double tol) const;
    // Health
    void updateHealth(double median, double tol, double timeMs);
    void markGood(ImuDataVars& d);
    void markBad(ImuDataVars& d);
    // Bias
    void learnBias();
};
}; //namespace yogurt