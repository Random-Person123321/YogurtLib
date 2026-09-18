#include "main.h"

/*
All electronic declarations
On a global scale
*/
//https://pros.cs.purdue.edu/v5/tutorials/topical/adi.html#line-tracker
// https://pros.cs.purdue.edu/v5/api/c/rtos.html#task-create
// https://pros.cs.purdue.edu/v5/tutorials/topical/multitasking.html





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() { //INIT SHOULD GO HERE!
    // startTime = pros::millis();
    // pros::lcd::initialize();
    // pros::lcd::print(5, "Imu Calibrated");
    //Start tasks, such as odom and intake stopping
    // start_tasks();
    //Calls auton distance relocalizations
    classInit();
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    //Need to add print to controller to here so prints happen pre-enable. Allows to check for IMU drift
}

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
void autonomous() {
    startTime = pros::millis();
    std::cout << "Autonomous Started!\n";

}

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
    /* REMOVED SO SKILLS DOESNT RUN FOR DRIVER, Re-enable if we want skillsto run
    startTime = pros::millis();
    std::cout << "Autonomous Started!\n";
    skills(); // Don't run skills for standard driver control
    */ 





    // pros::Motor frontintake(-7);
    // pros::Motor intake(17);
    // driver_skills();
    // resetOdometry(0.0, 0.0, 270.0); //Reset to 0,0,0 after testing is done
    // leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
    // leftMiddle.set_brake_mode(MOTOR_BRAKE_COAST);
    // leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
    // rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
    // rightMiddle.set_brake_mode(MOTOR_BRAKE_COAST);
    // rightBack.set_brake_mode(MOTOR_BRAKE_COAST);

    // //set pneumatics to default state
    // Blocker.set_value(false);
    // Matchload.set_value(false);
    // Mid.set_value(false);
    // Bunny.set_value(true);

    // //Matchload
    // bool Loading = false;
    // bool LastAButtonPressed = false;
    // //Bunny Ears
    // bool ears = false;
    // //Mid goal
    // bool mid = false;
    // bool LastL2ButtonPressed = false;
    // //Blocker
    // bool LastXButtonPressed = false;

    // bool toggle = false;
    // bool scoring = false;

    // while (true) {
    //     // Arcade control scheme
    //     int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
    //     int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
    //     left_mg.move(dir + turn);                      // Sets left motor voltage
    //     right_mg.move(dir - turn);                    // Sets right motor voltage

    //     // //Just blocker lift
    //     // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) ){
    //     //     Blocker.set_value(true);
    //     // } else {
    //     //     Blocker.set_value(false);
    //     // }

    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    //         intakes.move_voltage(12000);
    //         home = false;
    //         Blocker.set_value(true);
    //         scoring = true;
    //         toggle = false;
            
    //         if (mid == false) {
    //             if (cata.get_position() < startPosition + HcataRange) {
    //             catapult.move_voltage(12000);//Slow speed for testing
    //             } else {
    //             catapult.move_voltage(0);
    //             }
    //         }
    //         if (mid == true) {
    //             if (cata.get_position() < startPosition + LcataRange) {
    //             catapult.move_voltage(12000);//Slow speed for testing
    //             } else {
    //             catapult.move_voltage(0);
    //             }
    //         }
            
    //     }
    //     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ){
    //         intakes.move_voltage(12000);
    //         home = true;
    //         scoring = false;
    //         // Blocker.set_value(false);
    //     }
    //     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    //         home = true;
    //         scoring = false;
    //         // Blocker.set_value(false);
    //         intakes.move_voltage(-12000);
    //     }
    //     else {
    //         home = true;
    //         scoring = false;
    //         intakes.move_voltage(0);
    //         // Blocker.set_value(false);
    //     }

    //     //Just blocker lift
    //     bool currentXButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    //     if (currentXButton && !LastXButtonPressed){
    //         toggle = !toggle;
    //         // Blocker.set_value(block);
    //     }
    //     LastXButtonPressed = currentXButton;

    //     if (!toggle && !scoring) { //Both false
    //         Blocker.set_value(false);
    //     }
    //     else if (toggle || scoring) Blocker.set_value(true);
    //     /*
    //     else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    //         intakes.move_voltage(-5000);//slow for testing
    //         home = true;
    //     }
    //     else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    //         intakes.move_voltage(12000);
    //     }
    //     else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    //         intakes.move_voltage(8000);
    //     }
    //     else {
    //         intakes.move_voltage(0);
    //     }
    //     */
    //     //Bunny ear (Hold) Statements
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    //         Bunny.set_value(true);
    //     } else {
    //         Bunny.set_value(false);
    //     }
    //     // Intake lift and slow (Hold) outtake statements
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
    //         Lift.set_value(true);
    //         intakes.move_voltage(-3500);
    //     } else {
    //         Lift.set_value(false);
    //     }

    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    //         intakes.move_voltage(12000);

    //     }





    //     //Matchload If Statements
    //     bool CurrentAButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    //     if (CurrentAButton and !LastAButtonPressed) {
    //         Loading = !Loading;
    //         Matchload.set_value(Loading);
    //         // for (int i = 0; i < 3; i++) {
    //         //     distanceReset(LD, 70.5, 2.9, true, true);
    //         //     distanceReset(fD, -70.5, 9.2, false, false);
    //         // }
    //     }
    //     LastAButtonPressed = CurrentAButton;// Updates so it doesn't toggle back and force next loop
        

        

    //     bool CurrentL2Button = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    //     if (CurrentL2Button and !LastL2ButtonPressed) {
    //         mid = !mid;
    //         Mid.set_value(mid);
    //         // for (int i = 0; i < 3; i++) {
    //         //     distanceReset(rD, 70.5, 2.9, true, true);
    //         //     pros::delay(34);
    //         // }
    //     }
    //     LastL2ButtonPressed = CurrentL2Button;// Updates so it doesn't toggle back and force next loop


    //     // static uint32_t t = 0;
    //     // if (pros::millis() - t > 10) {
    //     //     t = pros::millis();
    //     //     master.print(0,0,"raw:%.0f pose:%.0f", 90 - imu1.get_heading(), pose.heading);
    //     // }

    //     for (int i = 0; i < imus.size(); i++) {
	// 		pros::lcd::print(i, "%.2lf", imus[i].get_heading());
	// 	}
    //     pros::delay(20); // Run for 20 ms then update
    // }
}
