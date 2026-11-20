#include "yogurtlib/api.hpp"
#include "globals.hpp"

double liftPercent(double per) {
    return per * liftMax / 100.0;
}
void rightRed() {
    pros::delay(2000);
    drivetrain.setPose(yogurt::Pose{62.0, 0.0, 0.0});
    TOGGLE_COLOUR = ToggleColour::RED;
    autoClawOn = false;
    liftPIDOn = false;
    autoToggleOn = true;
    liftPIDTarget = liftPercent(20.0);
    pros::delay(1000);
    autoToggleOn = false;
    liftPIDOn = true;
    drivetrain.driveStraight(-14.0, 1000 /*, {.minVolt = 2000.0, .earlyExitRange = 1.0}*/);

    drivetrain.turnToHeading(90.0, 800/*, {.minVolt = 2000.0, .earlyExitRange = 3.0}*/);
    drivetrain.driveStraight(-18.0, 1000, {.minVolt = 1000.0, .earlyExitRange = 1.0});
    drivetrain.turnToHeading(70.0, 500, {.minVolt = 2000, .earlyExitRange = 5.0});
    drivetrain.driveStraight(-4.0, 300, {.maxVolt = 5000.0}, true);
    claw.set_value(true);
    drivetrain.driveStraight(10.0, 1000, {.headingOffset = 30.0 - drivetrain.getPose().theta});
}