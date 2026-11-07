#include "yogurtlib/dt/dt.hpp"
using namespace yogurt;

void Drivetrain::setDistancesOffsets(DistanceSensors& distance, double xOffset, double yOffset){
    distance.xOffset = xOffset;
    distance.yOffset = yOffset;
}