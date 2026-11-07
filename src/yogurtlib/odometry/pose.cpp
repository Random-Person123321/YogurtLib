#include "yogurtlib/pose.hpp"
#include "yogurtlib/misc.hpp"
#include <cmath>

using namespace yogurt;

Pose::Pose(double x, double y, double theta) : x(x), y(y), theta(wrapDeg(theta)) {}

double Pose::distanceTo(const Pose& other) const {
        return std::hypot(other.x - x, other.y - y);
}

double Pose::angleTo(const Pose& other) const {
    return rad2deg(std::atan2(other.y - y, other.x - x));
}

double Pose::headingErrorTo(const Pose& other) const {
    return angleDiffDeg(angleTo(other), theta);
}

Pose Pose::operator+(const Pose& o) const { return Pose(x + o.x, y + o.y, theta + o.theta); }
Pose Pose::operator-(const Pose& o) const { return Pose(x - o.x, y - o.y, theta - o.theta); }
bool Pose::operator==(const Pose& o) const { return x == o.x && y == o.y && theta == o.theta; }
bool Pose::operator!=(const Pose& o) const { return !(*this == o); }
