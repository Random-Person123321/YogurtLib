#include "main.h"

void init_sensors() {
    catapult.set_brake_mode(MOTOR_BRAKE_COAST);

    left_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);

    imu1.reset(true);//Blocking for reliability
    imu2.reset(true);
    imu1.set_data_rate(5);
    imu2.set_data_rate(5);
    master.clear();


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
    printf("(Drive Temp: %.1lf Lever Temp: %.1lf Intake Temp: %.1lf)\n", drivetemp, levertemp, intaketemp); // Remove lever temp? (Replace with lift temp?)
}