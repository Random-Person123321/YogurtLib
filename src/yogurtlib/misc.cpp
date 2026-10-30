#include "yogurtlib/misc.hpp"
#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

void Drivetrain::tuneOffset() {
    requestMotion();
    double SdV_ccw = 0, SdV_cw = 0;//Sum of delta Vertical over turns
    double SdH_ccw = 0, SdH_cw = 0;
    double Stheta_ccw = 0, Stheta_cw = 0;

    //Need radians for formula
    auto spin = [&] (double lSpeed, double rSpeed, double angle, 
                    double& SdV, double& SdH, double& Stheta) {

        double H_IN_PER_TICK = (odom.horizontalDiameter * M_PI) / 36000.0;
        double V_IN_PER_TICK = (odom.verticalDiameter * M_PI) / 36000.0;
        double radAngle = deg2rad(angle);

        double lastHReading = odom.horizontal->get_position(), lastVReading = odom.vertical->get_position(), lastTheta = pose.theta;
        motors.left->move_voltage(lSpeed);
        motors.right->move_voltage(rSpeed);
        pros::delay(200);
        
        while (fabs(Stheta) < radAngle) {
            double currentHReading = odom.horizontal->get_position(), currentVReading = odom.vertical->get_position(), currentTheta = pose.theta;
            double thetaDeg = angleDiffDeg(currentTheta, lastTheta);

            SdV += (currentVReading - lastVReading) * V_IN_PER_TICK;
            SdH += (currentHReading - lastHReading) * H_IN_PER_TICK;
            Stheta += deg2rad(thetaDeg);

            lastTheta = currentTheta;
            lastHReading = currentHReading;
            lastVReading = currentVReading;
            pros::delay(10);
        }
    };
    spin(-7000, 7000, 720.0, SdV_ccw, SdH_ccw, Stheta_ccw);
    motors.left->move_voltage(0);
    motors.right->move_voltage(0);
    pros::delay(1000);
    spin(7000, -7000, 720.0, SdV_cw, SdH_cw, Stheta_cw);


    double denom = (Stheta_ccw - Stheta_cw);
    double horizontalOffset = (SdH_ccw - SdH_cw) / denom;
    double verticalOffset = (SdV_ccw - SdV_cw) / denom;
    printf("Vertical Offset: %.3lf\nHorizontal Offset: %.3lf\n", verticalOffset, horizontalOffset);
    motors.left->move_voltage(0);
    motors.right->move_voltage(0);
    pros::delay(1000);
    endMotion();
}

// *** Angle Helpers ***
double yogurt::wrapDeg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// returns shortest signed difference target - current in degrees
double yogurt::angleDiffDeg(double targetDeg, double currentDeg) {
  return wrapDeg(targetDeg - currentDeg);
}

double yogurt::deg2rad(double d) { return d * M_PI / 180.0; }

double yogurt::rad2deg(double r) { return r * 180.0 / M_PI; }

double yogurt::wrapRad(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// Convert PROS get_heading() [0,360] to signed [-180,180]
double yogurt::headingToSignedDeg(double h) {
  if (h > 180.0) h -= 360.0;
  return wrapDeg(h);
}