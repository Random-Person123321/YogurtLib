#include "main.h"
#include "yogurtlib/global.hpp"

// TODO-REMOVE: leftFront/leftMiddle/leftBack/rightFront/rightMiddle/rightBack
// were only ever used to build left_mg/right_mg below — dead now that
// left_mg/right_mg get built straight from ports when wiring up DriveMotors.
// pros::Motor leftFront(-16);
// pros::Motor leftMiddle(-13);
// pros::Motor leftBack(-14);
// pros::Motor rightFront(2);
// pros::Motor rightMiddle(3);
// pros::Motor rightBack(4);

// TODO-REMOVE: catapult/intake hardware — game-specific, not library material.
// pros::Motor intake(-11);
// pros::Motor catapult(20);

// TODO-PORT: these are exactly what DriveMotors(left, right, trackWidth,
// wheelDiameter, ...) needs — port into wherever the real `drivetrain`
// instance gets constructed (see main.cpp), don't leave as loose globals.
// pros::MotorGroup left_mg({-16, -13, -14});    // Creates a motor group with forwards ports 16 & 18 and reversed port 17
// pros::MotorGroup right_mg({2, 3, 4});  // Creates a motor group with forwards port 12 & 14 and reversed ports 13
// TODO-REMOVE: intakes motor group — game-specific.
// pros::MotorGroup intakes({-11}); // Create motor group for spinning all intakes

// TODO-REMOVE: pneumatics for last season's game mechanism — game-specific.
// pros::adi::DigitalOut Matchload('H');
// pros::adi::DigitalOut Blocker('E');
// pros::adi::DigitalOut Mid('G');
// pros::adi::DigitalOut Bunny('F');
// pros::adi::DigitalOut Lift('D');

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

// TODO-REMOVE: catapult/matchload-era state, game-specific — not template-
// library material (Parked, Last2ButtonPressed, activateJamDetection).
//Park Variables
bool Parked = false;
bool Last2ButtonPressed = false;

//Jam detect acivation
bool activateJamDetection = false;

// TODO-REMOVE (probably): generic match-timing bookkeeping, but check first
// whether anything besides the old drive.cpp/odometry.cpp still reads these
// (both of those are themselves marked for removal) before deleting.
//Time stuff (Clean?)
int startTime = 0;
int timeUsed = 0;

//Print time used
bool UseTime = false;

// RobotPose pose = {0.0, 0.0, 0.0};
// TODO-REMOVE: appears fully unused now — grep confirmed nothing reads
// odomOffset outside of this declaration and its `extern` in main.h.
double odomOffset = 0.0;

//NEW
//Just to help tie the different pieces of the library together
//Drivetrain is core library code
//UI & MCL is optional code the user can choose to include
yogurt::UI uiobject;

//Temporary
yogurt::DriveMotors hi(nullptr, nullptr, 15, 3.2, 1.0);
pros::Imu imu1(1);
pros::Imu imu2(2);
pros::Imu imu3(3);
std::vector<pros::v5::Imu *> imus = {&imu1, &imu2, &imu3};

yogurt::OdomSensors odometry(nullptr, 1.0, 0.0, nullptr, 1.0, 0.0, imus, nullptr, nullptr, nullptr, nullptr, 0.0, 0.0, 0.0, 0.0);
yogurt::ControllerSettings hi2(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 3);
yogurt::PursuitProfile hello3((double) 1, (double) 1, (double) 1, (double) 1, (double) 1, (double) 1, (double) 1, (double) 1, (double) 1);
// DriveMotors motors, OdomSensors odomSensors,
            //    ControllerSettings driveSettings, ControllerSettings angularSettings, //Lateral for driveStraight, Angular for heading correction,
            //    ControllerSettings turnSettings, ControllerSettings swingSettings, // turn for turn, swing for swing motion
            //    PursuitProfile pursuit, double maxVoltage = 12000.0, double defaultSlew = 800.0
yogurt::Drivetrain drivetrain(hi, odometry, hi2, hi2, hi2, hi2, hello3, 12, 1.0);

yogurt::UI task;
//Temporary Belongs in main.cpp if not for the precompiled binary

void classInit() {
    drivetrain.onCalibrated.push_back([](yogurt::Drivetrain& dt) {uiobject.screen_init(dt); });
    drivetrain.calibrate();
}