#pragma once

namespace yogurt {


struct Segment {
    double x1, x2;
    double y1, y2;
};

struct ParticleData {
    double x, y;
    double weight;
};

class MCL {
    // The distance from the center of your field to any inner wall (should be ~70.2in)
    void addWalls(double half);

    // The center of the box, then it's width and height along the x and y-axis
    void addBox(double cx, double cy, double width, double depth);

};

} // namespace yogurt