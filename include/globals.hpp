#pragma once
#include "main.h"
#include "yogurtlib/api.hpp"

extern yogurt::Drivetrain drivetrain;
extern pros::adi::DigitalOut toggle;
extern pros::adi::DigitalOut claw;

extern bool liftPIDOn;
extern int liftPIDTarget;
extern const int liftMax;
extern const int liftSpeed;

extern bool autoToggleOn;
extern bool autoClawOn;


inline void endPrint(yogurt::MotionResult result) {
    std::cout << yogurt::entostr(result) << '\n';
    yogurt::Pose p = drivetrain.getPose();
    std::cout << p.x << ' ';
    std::cout << p.y << ' ';
    std::cout << p.theta << '\n';
}
void lift();

enum ToggleColour {
    BLUE,
    RED
};

extern ToggleColour TOGGLE_COLOUR;

extern void liftHome();