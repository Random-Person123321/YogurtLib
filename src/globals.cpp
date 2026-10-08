#include "yogurtlib/global.hpp"

// TODO-PORT: exactly what OdomSensors.imus / OdomSensors.vertical /
// OdomSensors.horizontal need — port into the real `drivetrain`'s OdomSensors
// when it's constructed, don't leave as loose globals. `cata` (catapult
// rotation sensor) is game-specific — TODO-REMOVE.
// //All Sensors
// pros::Imu imu1(10);
// pros::Imu imu2(15);
// std::vector <pros::Imu> imus = {imu1, imu2};
// pros::Rotation verticalOdom(1);
// pros::Rotation horizontalOdom(22);
// pros::Rotation cata(19);
// TODO-PORT: OdomSensors.leftD/frontD/rightD/backD candidates (MCL will read
// these directly per the plan) — port when the real OdomSensors is built.
// pros::Distance bD(23);
// pros::Distance LD(17);
// pros::Distance fD(5);
// pros::Distance rD(6);

// TODO-PORT: needed by UI (uiobject) and by any opcontrol tank/arcade code —
// port into wherever globals get wired up for real, don't leave commented out.
// pros::Controller master(pros::E_CONTROLLER_MASTER);

// TODO-REMOVE (probably): generic match-timing bookkeeping, but check first
// whether anything besides the old drive.cpp/odometry.cpp still reads these
// (both of those are themselves marked for removal) before deleting.
//Time stuff (Clean?)
int startTime = 0;
int timeUsed = 0;

//Print time used
bool UseTime = false;

//NEW
//Just to help tie the different pieces of the library together
//Drivetrain is core library code
//UI & MCL is optional code the user can choose to include
yogurt::UI uiobject;

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
yogurt::ControllerSettings swings(250.0, 0.0, 0.0, 3.0, 0.75, 8.0, 200);
yogurt::PursuitProfile hello3{.curvSlowK = 1.75};
// DriveMotors motors, OdomSensors odomSensors,
            //    ControllerSettings driveSettings, ControllerSettings angularSettings, //Lateral for driveStraight, Angular for heading correction,
            //    ControllerSettings turnSettings, ControllerSettings swingSettings, // turn for turn, swing for swing motion
            //    PursuitProfile pursuit, double maxVoltage = 12000.0, double defaultSlew = 800.0
// yogurt::Drivetrain drivetrain(dm, odometry, forwards, forwardshead, turning, swings, hello3, 12000.0, 800.0);
yogurt::Drivetrain drivetrain(dm, odometry, forwards, turning, swings);

yogurt::UI task;
//Temporary Belongs in main.cpp if not for the precompiled binary

void classInit() {
    drivetrain.onCalibrated.push_back([](yogurt::Drivetrain& dt) {uiobject.screen_init(dt); });
    drivetrain.calibrate();
}