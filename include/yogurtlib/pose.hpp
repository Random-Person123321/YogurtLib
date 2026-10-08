#pragma once
#include "api.hpp"

namespace yogurt {

// x, y in inches; theta in degrees, wrapped to (-180, 180], math convention
// (0 = +X, +90 = +Y, CCW positive) to match the rest of the library.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Pose() = default;
    Pose(double x, double y, double theta = 0.0) : x(x), y(y), theta(wrapDeg(theta)) {}

    double distanceTo(const Pose& other) const {
        return std::hypot(other.x - x, other.y - y);
    }

    // Absolute heading (deg) of the straight line from this pose to other.
    double angleTo(const Pose& other) const {
        return rad2deg(std::atan2(other.y - y, other.x - x));
    }

    // Signed shortest turn (deg) needed to point this pose's heading at other.
    double headingErrorTo(const Pose& other) const {
        return angleDiffDeg(angleTo(other), theta);
    }

    Pose operator+(const Pose& o) const { return Pose(x + o.x, y + o.y, theta + o.theta); }
    Pose operator-(const Pose& o) const { return Pose(x - o.x, y - o.y, theta - o.theta); }
    bool operator==(const Pose& o) const { return x == o.x && y == o.y && theta == o.theta; }
    bool operator!=(const Pose& o) const { return !(*this == o); }
};

} // namespace yogurt
