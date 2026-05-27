#include "main.h"

pros::Motor leftFront(-16);
pros::Motor leftMiddle(-13);
pros::Motor leftBack(-14);
pros::Motor rightFront(2);
pros::Motor rightMiddle(3);
pros::Motor rightBack(4);

pros::Motor intake(-11);
pros::Motor catapult(20);

pros::MotorGroup left_mg({-16, -13, -14});    // Creates a motor group with forwards ports 16 & 18 and reversed port 17
pros::MotorGroup right_mg({2, 3, 4});  // Creates a motor group with forwards port 12 & 14 and reversed ports 13
pros::MotorGroup intakes({-11}); // Create motor group for spinning all intakes

pros::adi::DigitalOut Matchload('H');
pros::adi::DigitalOut Blocker('E');
pros::adi::DigitalOut Mid('G');
pros::adi::DigitalOut Bunny('F');
pros::adi::DigitalOut Lift('D');

//All Sensors 
pros::Imu imu1(10);
pros::Imu imu2(15);
std::vector <pros::Imu> imus = {imu1, imu2};
pros::Rotation verticalOdom(1);
pros::Rotation horizontalOdom(22);
pros::Rotation cata(19);
pros::Distance bD(23);
pros::Distance LD(17);
pros::Distance fD(5);
pros::Distance rD(6);

pros::Controller master(pros::E_CONTROLLER_MASTER);

//Park Variables
bool Parked = false;
bool Last2ButtonPressed = false;

//Jam detect acivation
bool activateJamDetection = false;

//Time stuff (Clean?)
int startTime = 0;
int timeUsed = 0;

//Print time used
bool UseTime = false;

RobotPose pose = {0.0, 0.0, 0.0};
double odomOffset = 0.0;