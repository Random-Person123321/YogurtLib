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
    bool benched = false;
    bool skip = false;
    bool dead = false;
    bool badRead = false;
    int rejects = 0;
    int goodstreak = 0;
    int badstreak = 0;
};


class IMU {
    
public:

   //IMU fusion VARIABLES:
    double inertialRejectTolerance = 5.0; //Degrees IMU's are allowed to drift before being excluded from averaging
    double inertialTurnLeniency = 0.05; //Percent that tolerance scales based on fast turns
    double maxTurnDelta = 720.0; //Maximum speed a IMU can measure, if an IMU reads above this it is ignored, in D/s
    int inertialAllowedRejects = 10; //Maximum rejects before an IMU is presumed "dead", once they pass this limit we do not care about their data any more
    double benchedImuDecay = 0.95; //The amount that each run decays the drift for IMU benching
    int benchedImuGoodRuns = 50; //How long of a good streak any benched IMU's must have before being allowed to rejoin the averaging
    int benchedImuBadRuns = 10; //How long of a bad streak any IMU's require to become benched

    std::vector<pros::Imu*> sensors;
    std::vector<yogurt::ImuDataVars> imuData;

    IMU(std::vector<pros::Imu*> sensors) : sensors(sensors) {imuData.resize(sensors.size()); }

    double lastFused = 0.0;
    double totalHeading = 0.0;
    
    double fuseIMUs(double time = 10.0) {
        double maxDegMs = maxTurnDelta / 1000.0 * time;
        double median = 0.0;
        double sorted = 0.0;
        int goodImus = 0;
        std::vector<std::pair<double, int>> sortedImu;

        for(int i = 0; i < sensors.size(); i++){
            imuData[i].skip = false; //Reset so no IMU's are skipped
            imuData[i].badRead = false;
        }

        if (sensors.size() >= 2){
            for (int i = 0; i < sensors.size(); i++) {
                double raw = sensors[i]->get_rotation();
                double reading = raw * imuData[i].scale;
                imuData[i].badRead = Imubad(reading);
                if (imuData[i].badRead) {imuData[i].skip = true; imuData[i].rejects += 1; continue;}
                if (!imuData[i].badRead) {imuData[i].delta = (imuData[i].last - raw) * imuData[i].scale; imuData[i].last = raw; }
                //Skip logic based on max speed (D/s the IMU can read befor being rejected)
                if(abs(imuData[i].delta) >= maxDegMs) {imuData[i].rejects += 1; imuData[i].skip = true; }
                if(imuData[i].rejects >= inertialAllowedRejects) imuData[i].dead = true;
                if (imuData[i].skip || imuData[i].dead || imuData[i].benched) continue;
                sortedImu.push_back({imuData[i].delta, i});
            }
            std::sort(sortedImu.begin(), sortedImu.end());
            int asize = sortedImu.size();
            if (asize == 0) return lastFused = 0.0;
            if (asize % 2 == 1) median = sortedImu[asize / 2].first;
            else median = (sortedImu[asize / 2 - 1].first + sortedImu[asize / 2].first) / 2.0;
            double tolerance = inertialRejectTolerance + (inertialTurnLeniency * fabs(median));
            for (int k = 0; k < asize; k++) {
                double d = sortedImu[k].first;    // the delta
                int idx  = sortedImu[k].second;   // which IMU it came from
                if (std::fabs(median - d) < tolerance) {
                    sorted += d;
                    goodImus += 1;
                } else {
                    imuData[idx].rejects += 1;
                }
            }
            lastFused = (goodImus > 0) ? sorted / goodImus : median;

            if (asize >= 3){
                //Go into benching logic
                for(int i = 0; i < sensors.size(); i++){
                    if(imuData[i].dead || imuData[i].skip) continue;
                    if(imuData[i].benched){
                        imuData[i].drift = (imuData[i].drift + (imuData[i].delta - lastFused)) * benchedImuDecay;
                        if (imuData[i].drift < inertialRejectTolerance) {imuData[i].goodstreak += 1;}
                        else {imuData[i].goodstreak = 0;}
                        if (imuData[i].goodstreak >= benchedImuGoodRuns) {imuData[i].goodstreak = 0; imuData[i].benched = false; imuData[i].badstreak = 0;}
                    } 
                    else if (!imuData[i].benched){
                        imuData[i].drift = (imuData[i].drift + (imuData[i].delta - lastFused)) * benchedImuDecay;
                        if(imuData[i].drift >= inertialRejectTolerance) {imuData[i].badstreak += 1;}
                        else {if(imuData[i].badstreak > 0) {imuData[i].badstreak -=1; } else {imuData[i].goodstreak += 1; }}
                        if (imuData[i].badstreak >= benchedImuBadRuns) {imuData[i].benched = true; imuData[i].goodstreak = 0; imuData[i].badstreak = 0;}
                    }
                }
            }

            return lastFused; //Return the fused and filtered imu data
        }
        else if (sensors.size() == 1){
            double raw = sensors[0]->get_rotation();
            if (Imubad(raw)){
                return 0; //Return 0 if the IMU reads inf or nan - assuming no heading change
            } else {
                double delta = imuData[0].last - raw;
                imuData[0].last = raw;
                lastFused = delta;
                return lastFused; //Return the difference in rotation - DEGREES
            }
        } else {
            return 0; //If 0 imu's or other errors
        }
    }

    bool Imubad (int value){
        return std::isnan(value) || std::isinf(value);
    }
};
}; //namespace yogurt