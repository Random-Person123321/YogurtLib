#include "yogurtlib/api.hpp"
#include "globals.hpp"


void leftBlue() {
    drivetrain.setPose(yogurt::Pose{62.0, 0.0, 0.0});
    TOGGLE_COLOUR = ToggleColour::BLUE;
    std::cout << "RED\n";
    autoClawOn = false;
    liftPIDOn = false;
    autoToggleOn = true;
    liftPIDTarget = liftPercent(24.0);
    pros::delay(1200);
    autoToggleOn = false;
    liftPIDOn = true;
    drivetrain.driveStraight(-14.0, 1000 /*, {.minVolt = 2000.0, .earlyExitRange = 1.0}*/);

    drivetrain.turnToHeading(90.0, 800/*, {.minVolt = 2000.0, .earlyExitRange = 3.0}*/);
    drivetrain.driveStraight(-18.0, 1000, {.minVolt = 1000.0, .earlyExitRange = 1.0});
    drivetrain.turnToHeading(70.0, 500, {.minVolt = 2000, .earlyExitRange = 5.0});
    drivetrain.driveStraight(-4.0, 500, {.maxVolt = 5000.0}, true);
    claw.set_value(true);
    drivetrain.driveStraight(9.8, 1000, {.headingOffset = 23.0 - drivetrain.getPose().theta});
    liftPIDTarget = liftPercent(6.7);
    autoClawOn = true;
    drivetrain.driveToPoint(30.75, -15.50, 600, 2000, {.maxVolt = 6600.0, .reversed = true}); //y:-17.5
    pros::delay(200);
    claw.set_value(false);
    autoClawOn = false;
    liftPIDTarget = liftPercent(42.0);
    drivetrain.turnToHeading(180.0, 1000, {.maxVolt = 4000.0});
    drivetrain.driveStraight(-21.5, 1000);
    claw.set_value(true);
    pros::delay(500);

    
    drivetrain.driveStraight(15.0, 700, {.minVolt = 3000, .earlyExitRange = 3.0});
    liftPIDTarget = liftPercent(6.7);
    drivetrain.turnToPoint(48.0, -36.9, 800, {.reversed = true });
    // // drivetrain.turnToHeading(100.0, 800.0);
    // // drivetrain.driveToPoint(48.0, -48.0, 400, 1000.0, {.reversed = true});
    autoClawOn = true;
    drivetrain.driveStraight(-22.5, 1000);
    pros::delay(200);
    claw.set_value(false);
    autoClawOn = false;
    pros::delay(200);
    liftPIDTarget = liftPercent(68);
    drivetrain.turnToHeading(-105.0, 1000.0, {.maxVolt = 4000.0});
    drivetrain.driveStraight(-15.50, 800, {.maxVolt = 8500.0});
    drivetrain.driveStraight(-5.0, 500, {.maxVolt = 4000.0});
    liftPIDTarget = liftPercent(50);
    claw.set_value(true);
    pros::delay(500);
    drivetrain.driveStraight(5.0, 1000);
}