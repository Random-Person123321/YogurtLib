#include "main.h"


//Asymetrical slew
// double slew(double target, double prev, double maxDelta) {
//     double delta = target - prev;
//     if (fabs(target) > fabs(prev) && fabs(delta) > maxDelta) {
//         // Limit acceleration toward the target without overshooting it
//         return prev + copysign(maxDelta, delta);
//     }
//     return target;  // Decel or within step; allow full change
// }

// TODO-PORT: genuinely useful, generic odometry-calibration routine (spins
// the robot to measure tracking-wheel offsets from center) — this belongs as
// a Drivetrain method (e.g. Drivetrain::tuneTrackingOffsets()), not thrown
// away. Don't delete until it has a home there.
// void tuneOffset() {
//     double SdV_ccw = 0, SdV_cw = 0;//Sum of delta Vertical over turns
//     double SdH_ccw = 0, SdH_cw = 0;
//     double Stheta_ccw = 0, Stheta_cw = 0;

//     //Need radians for formula
//     auto spin = [&] (double lSpeed, double rSpeed, double angle, 
//                     double& SdV, double& SdH, double& Stheta) {

//         double H_IN_PER_TICK = (HORIZ_DIAMETER * M_PI) / 36000.0;
//         double V_IN_PER_TICK = (VERT_DIAMETER * M_PI) / 36000.0;
//         double radAngle = deg2rad(angle);

//         double lastHReading = horizontalOdom.get_position(), lastVReading = verticalOdom.get_position(), lastTheta = getCurrentHeading();
//         left_mg.move_voltage(lSpeed);
//         right_mg.move_voltage(rSpeed);

//         while (fabs(Stheta) < radAngle) {
//             double currentHReading = horizontalOdom.get_position(), currentVReading = verticalOdom.get_position(), currentTheta = getCurrentHeading();
//             double thetaDeg = angleDiffDeg(currentTheta, lastTheta);

//             SdV += (currentVReading - lastVReading) * V_IN_PER_TICK;
//             SdH += (currentHReading - lastHReading) * H_IN_PER_TICK;
//             Stheta += deg2rad(thetaDeg);

//             lastTheta = currentTheta;
//             lastHReading = currentHReading;
//             lastVReading = currentVReading;
//             pros::delay(10);
//         }
//         left_mg.move_voltage(0);
//         right_mg.move_voltage(0);
//         // left_mg.brake();
//         // right_mg.brake();
//         pros::delay(300);
//     };
//     spin(-7000, 7000, 720.0, SdV_ccw, SdH_ccw, Stheta_ccw);
//     spin(7000, -7000, 720.0, SdV_cw, SdH_cw, Stheta_cw);

//     double denom = (Stheta_ccw - Stheta_cw);
//     double horizontalOffset = (SdH_ccw - SdH_cw) / denom;
//     double verticalOffset = (SdV_ccw - SdV_cw) / denom;
//     // pros::lcd::print(0, "Horizontal Offset: %.3lf", horizontalOffset);
//     // pros::lcd::print(1, "Vertical Offset: %.3lf", verticalOffset);
//     printf("Vertical Offset: %.3lf\n", verticalOffset);
// }

// TODO-PORT: still generically useful (FollowResult -> string for debug
// printing), but belongs as a yogurt:: namespaced free function near
// FollowResult/MotionResult (e.g. in control.hpp or dt.hpp) rather than
// living in misc.cpp. Low priority — not blocking anything.
std::string entostr (FollowResult result) {
    int num = (int)result;
    // switch (num){
    //     case 0: return "ReachEnd";
    //     case 1: return "Timeout";
    // }
    if (num == 0) {
        return "ReachedEnd";
    }
    else if (num == 1) {
        return "Timeout";
    }
    else if (num == 2) {
        return "NoIntersection_TargetBehind";
    }
    else if (num == 3) {
        return "Stuck";
    }
    else if (num == 4) {
        return "PathTooShort";
    }
    else return "Unknown";
}

