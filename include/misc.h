#ifndef MISC_H_
#define MISC_H_
#include <string>

void matchLoading(int time);

// double slew(double target, double prev, double maxDelta);

void tuneOffset();

void temps();

void voltages();

void inertials();

void printPose();

std::string entostr (FollowResult result);

#endif
