#pragma once

namespace yogurt {

// x, y in inches; theta in degrees, wrapped to (-180, 180], math convention
// (0 = +X, +90 = +Y, CCW positive) to match the rest of the library.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Pose() = default;
    Pose(double x, double y, double theta = 0.0);

    double distanceTo(const Pose& other) const;

    // Absolute heading (deg) of the straight line from this pose to other.
    double angleTo(const Pose& other) const;

    // Signed shortest turn (deg) needed to point this pose's heading at other.
    double headingErrorTo(const Pose& other) const;

    Pose operator+(const Pose& o) const;
    Pose operator-(const Pose& o) const;
    bool operator==(const Pose& o) const;
    bool operator!=(const Pose& o) const;
};

} // namespace yogurt
