#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/mcl.hpp"
using namespace yogurt;

void Drivetrain::setDistancesOffsets(DistanceSensors& distance, double xOffset, double yOffset) {
    distance.xOffset = xOffset;
    distance.yOffset = yOffset;
}

void FieldMap::addWalls(double half) {
    half = std::fabs(half); // Ensure value is positive
    double full = 2 * half;
    addBox(0.0,  0.0, full, full);

}

void FieldMap::addBox(double cx, double cy, double width, double depth) {
    // Push back into array of segments
    double top = cy - depth / 2, bottom = cy + depth / 2;
    double left = cx - width / 2, right = cx + width / 2;
    //     Add Segment          x1,   y1,  x2,   y2
    fieldMap.push_back(Segment{left, top, right, top}); // Top wall
    fieldMap.push_back(Segment{left, top, left, bottom}); // Left wall
    fieldMap.push_back(Segment{right, top, right, bottom}); // Right wall
    fieldMap.push_back(Segment{left, bottom, right, bottom}); // Bottom wall
}

const std::vector<Segment>& FieldMap::segment() const {return fieldMap; }

double FieldMap::rayCast(double sensorX, double sensorY, double dirX, double dirY) const {
    double best = -1.0;
    for (auto &seg : fieldMap) {
        double SegX = seg.x2 - seg.x1, SegY = seg.y2 - seg.y1;
        double denom = dirX * SegY - dirY * SegX;
        if (std::fabs(denom) < 1e-9) continue;
        double toStartX = seg.x1 - sensorX, toStartY = seg.y1 - sensorY;
        double beamDist  = (toStartX * SegY - toStartY * SegX) / denom;
        double segFrac = (toStartX * dirY - toStartY * dirX) / denom;
        if (beamDist > 0 && segFrac >= 0 && segFrac <= 1) {
            best = (best < 0.0) ? beamDist  : (beamDist < best ? beamDist : best);
        }
    }
    return best;
}