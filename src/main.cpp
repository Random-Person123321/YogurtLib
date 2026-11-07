#include "main.h"
#include "yogurtlib/api.hpp"

// yogurt::Drivetrain drivetrain;

pros::MotorGroup r_motors ({-18, 17});
pros::MotorGroup l_motors ({-19, 20});

//Temporary
yogurt::DriveMotors dm(&l_motors, &r_motors, 2.75);
pros::Imu imu1(1);
pros::Imu imu2(2);
pros::Imu imu3(3);
std::vector<pros::v5::Imu *> imus = {&imu1, &imu2, &imu3};

pros::Rotation vert(4);
pros::Rotation horz(5);

yogurt::OdomSensors odometry(&vert, 2.071, 0.256, &horz, 1.99, -2.718, imus, nullptr, nullptr, nullptr, nullptr, 0.0, 0.0, 0.0, 0.0);
yogurt::ControllerSettings forwards(600.0, 0.0, 0.0, 3.0, 0.1, 0.5, 1000);
yogurt::ControllerSettings forwardshead(300.0, 0.0, 5.0, 2.0, 0.5, 1.0, 250);
yogurt::ControllerSettings turning(120.0, 0.0, 15.35, 2.0, 0.75, 8.0, 200);
yogurt::ControllerSettings swings(200.0, 0.0, 16.0, 3.0, 0.75, 8.0, 200); //THIS DOENST NEED TOLERANCE
yogurt::PursuitProfile hello3{};
// DriveMotors motors, OdomSensors odomSensors,
            //    ControllerSettings driveSettings, ControllerSettings angularSettings, //Lateral for driveStraight, Angular for heading correction,
            //    ControllerSettings turnSettings, ControllerSettings swingSettings, // turn for turn, swing for swing motion
            //    PursuitProfile pursuit, double maxVoltage = 12000.0, double defaultSlew = 800.0
// yogurt::Drivetrain drivetrain(dm, odometry, forwards, forwardshead, turning, swings, hello3, 12000.0, 800.0);
yogurt::Drivetrain drivetrain(dm, odometry, forwards, turning, swings);

yogurt::UI uiobject;

pros::Controller master(CONTROLLER_MASTER);

// void classInit() {
//     drivetrain.onCalibrated.push_back([](yogurt::Drivetrain& dt) {uiobject.screen_init(dt); });
//     drivetrain.calibrate();
    
// }
void classInit() {
    // drivetrain.setHeadingCorrectionGains(25.0, 0.0, 0.0, 3.0);
    drivetrain.setHeadingCorrectionGains(430.0, 0.0, 26.875, 2.0);
    drivetrain.setBoomerangSettleGains({}, {});
    drivetrain.onCalibrated.push_back([](yogurt::Drivetrain& dt) {uiobject.screen_init(dt); });
    drivetrain.calibrate();
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {classInit();}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    while(true){
        drivetrain.motors.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X), true, 2.3);
        pros::delay(20);
    }
} 