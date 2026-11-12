#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/mcl.hpp"
using namespace yogurt;

void Drivetrain::setDistancesOffsets(DistanceSensors& distance, double xOffset, double yOffset) {
    distance.xOffset = xOffset;
    distance.yOffset = yOffset;
}

void MCL::addWalls(double half) {

}