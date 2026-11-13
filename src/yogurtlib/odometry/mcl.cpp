#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/mcl.hpp"
using namespace yogurt;

void Drivetrain::setDistancesOffsets(DistanceSensors& distance, double xOffset, double yOffset) {
    distance.xOffset = xOffset;
    distance.yOffset = yOffset;
}

void FieldMap::addWalls(double half) {
    half = std::fabs(half); // Ensure value is positive
    double centeredHalf = half + 1.0; double doubleHalf = 2 * half;
    //      center-x       center-y      width       depth
    addBox( centeredHalf,  0.0,          2.0,        doubleHalf);
    addBox(-centeredHalf,  0.0,          2.0,        doubleHalf);
    addBox( 0.0,           centeredHalf, doubleHalf, 2.0);
    addBox( 0.0,          -centeredHalf, doubleHalf, 2.0);
}

void FieldMap::addBox(double cx, double cy, double width, double depth) {
    // Push back into array of segments
    fieldMap.push_back(Segment{ //    x1,            y1
                                cx - width / 2, cy + depth / 2,
                                //    x2,            y2
                                cx + width / 2, cy - depth / 2 });
}

const std::vector<Segment>& FieldMap::segment() {return fieldMap; }

void FieldMap::rayCast(double sensorX, double sensorY, double dirX, double dirY) const {
    double best = -1.0;
    for (auto &seg : fieldMap) {
        double SegX = seg.x2 - seg.x1, segY = seg.y2 - seg.y1;
        double denom = dirX * segY - dirY * SegX;
        if (std::fabs(denom) < 1e-9) continue;
        double wx = seg.x1 - sensorX, wy = seg.y1 - sensorY;
        double beamDist  = (wx * segY - wy * SegX) / denom;
        double segFrac = (wx * dirY - wy * dirX) / denom;
    }
}