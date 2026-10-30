#pragma once

#include <algorithm>
#include "yogurtlib/pose.hpp"
namespace yogurt{

struct PathVec {
    double first, second;
};

struct PathPoint {
    double x = 0.0, y = 0.0;
    double s = 0.0; // How many inches along the path the robot is
    double curvature = 0.0, vel = 0.0; // Curvature and velocity (0-127)
};

struct PathConfig {
    double maxSpeed = 127.0; //max speed the robot drives, out of 127
    double decelDist = 24.0; //Inches that allows the robot to slow from full speed to 0. Smaller number means braking later
    double turnK = 5.0; //speed(0-127) per inch of turn radius. How much to slow down by during tight curves/turns
    double maxPtSpacing = 3.0; /* If any segment is more than 3" apart, the path build() function will inject more.
                                  Useful for driveToPoint function where there is only two largely space points */
};

struct WheelTargets {
    double leftVel = 0.0, rightVel = 0.0;
    double leftAccel = 0.0, rightAccel = 0.0;
};

struct VoltageTerms {
    double kV = 95.0;
    double kA = 0.0;
    double kS = 0.0; //Tune
    double kP = 0.0; //Tune
};

struct FollowReturn {
    WheelTargets wheels;
    bool done = false, pivoting = false;
    double distToPath = 0.0;
};


class Path {
private:
    std::vector<PathVec> waypoints; //Just uses x and y coordinates
    std::vector<PathPoint> pts;
    PathConfig configs;
    bool startReplaced = false;
    double trackWidth = 0.0; // inches, center-to-center of left/right wheels

    static double distance (const PathPoint& first, const PathPoint& second);

    // Builds the path with all the functions inside the Path class, 
    // this includes standard speed calculations, curvature calculations and
    // ensuring the path is valid to drive
    void build();

public: //Const accepts written in lists like {{0, 0}, {10, 5}}

    Path (const std::vector<PathVec>& waypoints, const PathConfig& configs);

    /**
     * @brief Sets the first point of the path to the current robot pose and calls build()
     */    
    void setStart(double x, double y);

    /**
     * @brief Sets the PurePursuit track width (based on Drivetrain track width), THIS ONLY SETS FOR PUREPURSUIT AND IS OVERWRITTEN - USE DRIVETRAINS TRACK WIDTH
     */
    void setTrackwidth(double tw);
    
    /**
     * @brief Returns the trackwidth from purepursuit
    */
    double getTrackWidth() const;

    const PathPoint& operator[](size_t i) const;

    /**
     * @brief returns the length of a built path
    */
    int size() const;
    
    /**
     * @brief returns if a path is valid (>2 points)
    */
    bool valid() const;
    
    const PathPoint& back() const;

    /**
     * @brief returns the total length of the path from calculated values
    */
    double length() const;

    const PathConfig& config() const;
};

class PurePursuit {
public:
    PurePursuit(const Path& path, double startSpeed, double startL, double startR);
    
    /**
     * @brief the most imporant part of the purepursuit implimentation, it strings together the functions in th PurePursuit class and returns for the drivetrain to follow, Rough steps include:
     * 
     * -  Updating the closest point to the robot based on current pose
     * 
     * -  Finding the lookahead point
     *
     * - Finding the curveature of the path based on the current pose, last and next point using the quadratic formula
     *
     * - Find the target speed based on the curvature, and convert to drivetrain sides
     *
     * - Check if the robot has cross a perpendicular line with the final point in the path
     *
     * - Update variables for Async
    */
    FollowReturn step(Pose& p, double lookaheadDist, bool reversed, 
                      double minSpeed, double maxStep, double endTolerance, double dt);

    /**
     * @brief returns the closest point from the path to the robot
    */
    size_t closest() const;
    
    /**
     * @brief returns the compaired value between the total length and how far the robot has driven so far (Useful for async)
    */
    double getProgress() const;
    
private:
    Path path;
    size_t closestIdx = 0; //Closest point, always moves in direction of travel
    double progressS = 0.0;
    double lastLookFrac = 0.0;
    double prevVel = 0.0;
    double lookX = 0.0, lookY = 0.0;
    double prevL = 0.0, prevR = 0.0;

    // Update the closest point from the path based on lookahead
    size_t updateClosest(double x, double y, double lookaheadDist);

    // Check if something is in the lookahead dist
    bool findLookahead(double x, double y, double L);

    // Calculate curvature from the current pose, instead of theoretical
    double arcCurv(Pose& p, bool reversed, bool& behind, double& side);

    // Figure out the speed based on the path ideal
    double targetSpeed(Pose& pose, double minSpeed, double maxStep);

    // Take speed, direction and real curvature to determine how fast each side of the drivetrain needs to move
    WheelTargets wheelSpeeds(double target, double curv, bool reversed, double dt, bool behind, double side);

    // Check if we are passsed the endpoint of the path
    bool isDone(const Pose& pose, double endTolerance, double lookaheadDist) const;
};
} //namespace yogurt