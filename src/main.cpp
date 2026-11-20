#include "main.h"
#include "yogurtlib/api.hpp"
#include "yogurtlib/misc.hpp"
#include "globals.hpp"
#include "autons.hpp"

//Just to help tie the different pieces of the library together
//Drivetrain is core library code
//UI & MCL is optional code the user can choose to include
yogurt::UI uiobject;

pros::Controller master (CONTROLLER_MASTER);
pros::MotorGroup r_motors ({-20, 19});
pros::MotorGroup l_motors ({-17, 18});
pros::MotorGroup winch ({-15, 16});
pros::MotorGroup toggles ({-14, 13});

pros::adi::DigitalOut claw ('H');
pros::adi::DigitalOut rake ('G');

yogurt::DriveMotors dm (&l_motors, &r_motors, 2.75);

// Sensors
pros::Imu imu1(1);
pros::Imu imu2(2);
pros::Imu imu3(3);
yogurt::IMU imus({&imu1, &imu2, &imu3});

pros::Rotation vert(-4);
pros::Rotation horz(5);
// WINCH Rotation
pros::Rotation liftEnc(-11);

pros::Distance clawdist(10);

pros::Optical colour(12);


yogurt::OdomSensors odometry(&vert, 1.99, 0.3848, &horz, 1.99, -4.8545, imus, nullptr, nullptr, nullptr, nullptr, 0.0, 0.0, 0.0, 0.0);
yogurt::ControllerSettings forwards(620.0, 0.0, 18.0, 2.0, 0.75, 1.0, 250);
yogurt::ControllerSettings turning(220.0, 0.0, 10.0, 2.0, 0.75, 8.0, 250); //0.75 tol, 13kd
yogurt::ControllerSettings swings(320.0, 0.0, 24.5, 2.0, 0.75, 8.0, 250);

yogurt::Drivetrain drivetrain(dm, odometry, forwards, turning, swings);

uint32_t startTime;



ToggleColour TOGGLE_COLOUR = ToggleColour::BLUE;

bool toggleOn = false;
bool liftPIDOn = false;
int liftPIDTarget = 0;
const int liftMax = 255648;
const int liftSpeed = 1500; //350, 500
// yogurt::PID liftPID(9, 1.52, 13.3, 10.0);
yogurt::PID liftPID(60.0, 0.0, 0.0, 10.0);

void lift() {
    liftPID.reset();
     // yogurt::PID liftPID(1.6, 0.0, 0.01, 2.0);
    double prevLift = 0.0;
    int cnt = 0;
    bool settled = false;
    while (true) {
        if (!liftPIDOn) {pros::delay(20); continue; }
        int currentEnc = liftEnc.get_position();
        liftPIDTarget = std::clamp(liftPIDTarget, 0, liftMax);
        int setPoint = std::clamp(liftPIDTarget, currentEnc - 20000, currentEnc + 20000);
        double liftErr = (setPoint - currentEnc) / 100.0;
        double a = std::fabs(liftErr);
        if (!settled && a < 5.0) settled = true; //5 degrees of tolerance
        if (settled && a > 10.0) settled = false;
        if (settled) {liftErr = 0.0; prevLift = 0.0; } // Tolerance
        
        double liftOut = liftPID.calculateError(liftErr, 0.02);
        double kS = liftErr > 0 ? 2500.0 : 1500.0;
        if(liftErr != 0) liftOut += std::copysign(kS, liftErr);
        if (liftErr < 0) liftOut *= 0.6667;
        liftOut = std::clamp(liftOut, -12000.0, 12000.0);
        liftOut = yogurt::Drivetrain::slew(liftOut, prevLift, 1500.0);
        prevLift = liftOut;
        
        cnt++;
        if (cnt == 10) {
            cnt = 0;
            // printf("Tar: %d, Enc: %d, Err: %d, Out: %.1lf\n", liftPIDTarget, currentEnc, liftErr, liftOut);
            // printf("Tar: %d\n", currentEnc);
            // auto current = winch.get_current_draw_all();
            // auto vel = winch.get_actual_velocity_all();
            // printf("Vel:%.1lf ", (vel[0] + vel[1]) / 2);
            // printf("mA:%d\n",(current[0] + current[1]) / 2);
            fflush(stdout);
        }
        // std::cout << liftErr << " " << liftOut << '\n'
        // printf("%d,%d,%.1f,%.1f\n", liftPIDTarget, currentEnc, liftErr, liftOut);
        winch.move_voltage(liftOut);
        if (!toggleOn) toggles.move_voltage(liftOut);
        pros::delay(20);
    }
}

void liftHome() {
    winch.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    toggles.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    int time = 0;
    int cnt = 0;
    winch.move_voltage(12000);
    toggles.move_voltage(12000);
    while (time < 2500) {
        auto current = winch.get_current_draw_all();
        auto vel = winch.get_actual_velocity_all();
        if ((current[0] + current[1]) / 2 > 1800 &&
        (vel[0] + vel[1]) / 2 < 20){
            liftEnc.set_position(liftMax);
            printf("Homing Done\n");
            return;
        }
        cnt++;
        if (cnt == 20) {
            printf("%d\n", (current[0] + current[1]) / 2);
            printf("%.1lf\n", (vel[0] + vel[1]) / 2);
            cnt = 0;
        }
        pros::delay(10);
        time += 10;
    }
    printf("Homing TIMEOUT\n");
}
bool useAutoClaw = true;


bool autoClaw = false;
bool autoClawOn = false;
void autoClawTask() {
    while(true) {
        double dist = clawdist.get_distance();
        if (dist <= 50.0 /*&& !master.get_digital(DIGITAL_R2)*/) { //35
            autoClaw = true;
        }
        else {
            autoClaw = false;
        }
        pros::delay(20);
    }
}

bool autoToggleOn = false;
void autoToggle() {
    double reading = colour.get_hue();
    while(true) {
        if (!autoToggleOn) {pros::delay(20); continue; }
        if(master.get_digital(DIGITAL_Y) || pros::competition::is_autonomous()) {
            colour.set_led_pwm(50);
            reading = colour.get_hue();
            if (TOGGLE_COLOUR == ToggleColour::BLUE){
                double target = (180 + 240) / 2;
                if (std::fabs(yogurt::angleDiffDeg(target, reading)) > 30) {
                    toggles.move_voltage(-8200);
                    toggleOn = 1;
                }
                else {pros::delay(100); toggles.move_voltage(0); toggleOn = 0; }
            }
            else {
                if (std::fabs(yogurt::angleDiffDeg(7.0, reading)) >  10) {
                    toggles.move_voltage(-8200);
                    toggleOn = 1;
                }
                else {pros::delay(100); toggles.move_voltage(0); toggleOn = 0; }
            }
        } else {
            colour.set_led_pwm(0);
            toggleOn = 0;
        }
        pros::delay(20);
    }
}

void raiseLift() {
    winch.move_voltage(12000);
    toggles.move_voltage(12000);
    pros::delay(400);
    winch.move_voltage(0);
    toggles.move_voltage(0);
}

void classInit() {
    // drivetrain.setHeadingCorrectionGains(25.0, 0.0, 0.0, 3.0);
    drivetrain.setHeadingCorrectionGains(140.0, 0.0, 6.5, 2.0); //x120
    drivetrain.setBoomerangSettleGains({}, {});
    drivetrain.onCalibrated.push_back([](yogurt::Drivetrain& dt) {uiobject.screen_init(dt); });
    drivetrain.odom.imus.setScale(0, 0.9976645);
    drivetrain.odom.imus.setScale(1, 1.0004675);
    drivetrain.odom.imus.setScale(2, 0.9910535);
    drivetrain.calibrate();
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() { //INIT SHOULD GO HERE!
    pros::Task toggleTask(autoToggle);
    pros::Task liftTask(lift);
    pros::Task clawtask(autoClawTask);
    classInit();
    colour.set_led_pwm(0);
    liftEnc.set_data_rate(5);
    claw.set_value(false);
    liftHome();
}


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

void debug() {
    while (true) {
        yogurt::Pose p = drivetrain.getPose();
        // printf("(X: %.1lf, Y: %.1lf, H: %.1lf, MU1: %.1lf, MU2: %.1lf, MU3: %.1lf)\n", p.x, p.y, p.theta, drivetrain.odom.imus.sensors[0]->get_heading(), drivetrain.odom.imus.sensors[1]->get_heading(), drivetrain.odom.imus.sensors[2]->get_heading());
        printf("(X: %.1lf, Y: %.1lf, H: %.1lf, T: %d) ", p.x, p.y, p.theta, pros::millis() - startTime);

        std::vector<double> raw = winch.get_position_all();
        double avg = 0.0;
        for (int i = 0; i < raw.size(); i++){
            avg += raw[i];
        }
        avg = avg / raw.size();
        printf("Enc: %.1lf\n", avg);

        pros::delay(200);
    }
}

yogurt::Pose yogurt::Drivetrain::getUnwrappedPose() const {
    poseMutex.take();
    Pose p = pose;
    poseMutex.give();
    return p;
}

void autonomous() {
    yogurt::MotionResult result;
    pros::Task task(debug);
    startTime = pros::millis();
    // drivetrain.driveStraight(12.0, 1000);
    // drivetrain.turnToHeading(300.0, 1000, {.dir = yogurt::TurnDirection::CLOCKWISE});
    // result = drivetrain.turnToHeading(180.0, 6000.0);
    // std::cout << yogurt::entostr(result);
    // result = drivetrain.turnToHeading(90.0, 6000.0);
    // std::cout << yogurt::entostr(result);
    rightRed();
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
    // pros::Task task(debug);
    // liftHome();


    liftPIDOn = true;
    autoClawOn = true;
    autoToggleOn = true;
    // liftPIDOn = false;
    // liftPIDTarget = 112984;
    
    
    // bool LastYButtonPressed = false;
    // bool lastBButonPressed = false;
    bool lastR1ButonPressed = false;
    bool lastR2ButonPressed = false;
    bool LastRButtonPressed = false;
    bool rakeMode = false;
    bool clawClose = true;
    bool lastClawState = false;

    int cnt = 0;
    while(true) {
        drivetrain.motors.arcade(
            master.get_analog(ANALOG_LEFT_Y),
            master.get_analog(ANALOG_RIGHT_X), true, 1.6);
            
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftPIDTarget += liftSpeed;
                // winch.move_voltage(12000);
                // toggles.move_voltage(12000);
            }
            else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftPIDTarget -= liftSpeed;
                // winch.move_voltage(-8000);
                // toggles.move_voltage(-8000);
            }
            else {
                // winch.move_voltage(0);
                // toggles.move_voltage(toggleOn? -8200 : 0);
            }

            bool CurrentRButton = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
            if (CurrentRButton and !LastRButtonPressed) {
                rakeMode = !rakeMode;
                rake.set_value(rakeMode);
            }
            LastRButtonPressed = CurrentRButton;// Updates so it doesn't toggle back and force next loop

            // bool currentBButton = master.get_digital(DIGITAL_B);
            // if (currentBButton && lastBButonPressed != currentBButton) {
            //     clawClose = !clawClose;
            // }
            // lastBButonPressed = currentBButton;
            bool currentR1Button = master.get_digital(DIGITAL_R1);
            if (currentR1Button && lastR1ButonPressed != currentR1Button) {
                autoClawOn = true;
                clawClose = !clawClose;
            }
            lastR1ButonPressed = currentR1Button;
            int print = -1;
            if (autoClawOn && autoClaw && clawClose) { // If they both agree to close, then close
                print = 0;
                // claw.set_value(false); //false is close, true is open
                
                // if (lastClawState) {
                //     print = 1;
                //     raiseLift(); 
                //     pros::delay(10);
                // }
                lastClawState = false;
            }
            else if (autoClawOn && autoClaw && !clawClose) { 
                // claw.set_value(true); // If the user wants it open, they get it
                print = 2;
                lastClawState = true;
            }
            else if (autoClawOn && !autoClaw && !clawClose) {/*claw.set_value(true);*/ lastClawState = true; print = 3; } // same as previus

            bool currentR2Button = master.get_digital(DIGITAL_R2);
            if (currentR2Button && lastR2ButonPressed != currentR2Button) {
                autoClawOn = false;
                lastClawState = !lastClawState;
                // claw.set_value(lastClawState);
            }
            lastR2ButonPressed = currentR2Button;

            claw.set_value(lastClawState);
        if (++cnt % 40 == 0) {
            auto current = winch.get_current_draw_all();
            auto vel = winch.get_actual_velocity_all();
            // std::cout << lastClawState;
            
            // if (print == 0) std::cout << "YES ";
            // else if (print == 1) std::cout << "start ";
            // else if (print == 2)  std::cout << "OPEN User override\n";
            // else if (print == 3) std::cout << "Open\n";
            // std:: cout << liftEnc.get_position() << '\n';
            // std::cout << autoClaw << " " << clawClose << '\n';
            // printf("Vel:%.1lf ", (vel[0] + vel[1]) / 2);
            // printf("mA:%d\n",(current[0] + current[1]) / 2);
        }
    //     // winch.move_voltage(12000);
        pros::delay(5);
    }
}