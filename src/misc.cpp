#include "main.h"

void antiJam(void* param) {
    while(true) {
        if(activateJamDetection) {
            double m = intake.get_actual_velocity();//70, 63
            int cur = intake.get_current_draw();
            int dir = intake.get_direction();
            if (m < 60.0 && dir == 1 && cur > 2000) {
                intakes.move_voltage(-8000);
                pros::delay(200);
                intakes.move_voltage(12000);
                pros::delay(50);
            }
        }
        pros::delay(20);
    }
}


void matchLoading(int time) {
    int T = 0;
    while(T < time) {
        driveStraight(67, 200, 0.0, 12000);
        T += 200;
    }
}

//Asymetrical slew
double slew(double target, double prev, double maxDelta) {
    double delta = target - prev;
    if (fabs(target) > fabs(prev) && fabs(delta) > maxDelta) {
        // Limit acceleration toward the target without overshooting it
        return prev + copysign(maxDelta, delta);
    }
    return target;  // Decel or within step; allow full change
}

void tuneOffset() {
    double SdV_ccw = 0, SdV_cw = 0;//Sum of delta Vertical over turns
    double SdH_ccw = 0, SdH_cw = 0;
    double Stheta_ccw = 0, Stheta_cw = 0;

    //Need radians for formula
    auto spin = [&] (double lSpeed, double rSpeed, double angle, 
                    double& SdV, double& SdH, double& Stheta) {

        double H_IN_PER_TICK = (HORIZ_DIAMETER * M_PI) / 36000.0;
        double V_IN_PER_TICK = (VERT_DIAMETER * M_PI) / 36000.0;
        double radAngle = deg2rad(angle);

        double lastHReading = horizontalOdom.get_position(), lastVReading = verticalOdom.get_position(), lastTheta = getCurrentHeading();
        left_mg.move_voltage(lSpeed);
        right_mg.move_voltage(rSpeed);

        while (fabs(Stheta) < radAngle) {
            double currentHReading = horizontalOdom.get_position(), currentVReading = verticalOdom.get_position(), currentTheta = getCurrentHeading();
            double thetaDeg = angleDiffDeg(currentTheta, lastTheta);

            SdV += (currentVReading - lastVReading) * V_IN_PER_TICK;
            SdH += (currentHReading - lastHReading) * H_IN_PER_TICK;
            Stheta += deg2rad(thetaDeg);

            lastTheta = currentTheta;
            lastHReading = currentHReading;
            lastVReading = currentVReading;
            pros::delay(10);
        }
        left_mg.move_voltage(0);
        right_mg.move_voltage(0);
        // left_mg.brake();
        // right_mg.brake();
        pros::delay(300);
    };
    spin(-7000, 7000, 720.0, SdV_ccw, SdH_ccw, Stheta_ccw);
    spin(7000, -7000, 720.0, SdV_cw, SdH_cw, Stheta_cw);

    double denom = (Stheta_ccw - Stheta_cw);
    double horizontalOffset = (SdH_ccw - SdH_cw) / denom;
    double verticalOffset = (SdV_ccw - SdV_cw) / denom;
    // pros::lcd::print(0, "Horizontal Offset: %.3lf", horizontalOffset);
    // pros::lcd::print(1, "Vertical Offset: %.3lf", verticalOffset);
    printf("Vertical Offset: %.3lf\n", verticalOffset);
}

std::string entostr (FollowResult result) {
    int num = (int)result;
    // switch (num){
    //     case 0: return "ReachedEnd";
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



void temps () {
    double drivetemp = 0.0;
    double levertemp = 0.0;
    double intaketemp = 0.0;

    std::vector<double> left_drive = left_mg.get_temperature_all();
    std::vector<double> right_drive = right_mg.get_temperature_all();
    double sum = 0;
    for (auto i : left_drive) {
        sum += i;
    }
    for (auto i : right_drive) {
        sum += i;
    }
    drivetemp = sum / 6;
    levertemp = catapult.get_temperature();
    intaketemp = intake.get_temperature();
    master.print(0, 0, "DT:%.0lf, L:%.0lf  I:%.0lf ", drivetemp, levertemp, intaketemp);
    // master.print(0, 0, "Drive:%.0lf", drivetemp);
    // master.print(1, 0, "Lever:%.0lf", levertemp);
    // master.print(2, 0, "Intake:%.0lf", intaketemp);
}

void voltages () {
    double LdriveVoltage = 0.0, RdriveVoltage = 0.0;

    std::vector<long> left_drive = left_mg.get_voltage_all();
    std::vector<long> right_drive = right_mg.get_voltage_all();
    long Lsum = 0, Rsum = 0;
    for (auto i : left_drive) {
        Lsum += i;
    }
    for (auto i : right_drive) {
        Rsum += i;
    }
    LdriveVoltage = Lsum / 3.0;
    RdriveVoltage = Rsum / 3.0;

    master.print(1, 0, "LV: %5.0lf, RV:% 5.0lf", LdriveVoltage, RdriveVoltage);
}

void inertials () {
    master.print(2, 0, "MU1:% 3.0lf, MU2:% 3.0lf", imu1.get_heading(), imu2.get_heading());
}

void printPose() {
    master.print(1, 0, "X: %.1lf Y: %.1lf H: %.1lf", pose.x, pose.y, pose.heading);
}
