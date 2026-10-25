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

inline double slew(double target, double prev, double maxDelta) {
        if (std::signbit(target) != std::signbit(prev) || target == 0.0) prev = 0.0; // A change of direction counts as braking
        double delta = target - prev; // A slowdown towards zero is free
        if (fabs(target) > fabs(prev) && fabs(delta) > maxDelta) {
            // Limit acceleration toward the target without overshooting it
            return prev + copysign(maxDelta, delta);
        }
        return target;  // Decel or within step; allow full change
}

class Path {
private:
    std::vector<PathVec> waypoints; //Just uses x and y coordinates
    std::vector<PathPoint> pts;
    PathConfig configs;
    bool startReplaced = false;
    double trackWidth = 0.0; // inches, center-to-center of left/right wheels

    static double distance (const PathPoint& first, const PathPoint& second) {
        double dx = second.x - first.x;
        double dy = second.y - first.y;
        return std::hypot(dx, dy);

    }
    void build() {
        pts.clear();
        std::vector<PathPoint> raw = {};
        if (waypoints.size() < 2) return;
        for (size_t i = 0; i < waypoints.size(); i++) {
            PathPoint p = {waypoints[i].first, waypoints[i].second};
            if (raw.empty()) {
                raw.push_back(p);
                continue;
            }
            
            double dx = waypoints[i].first - raw.back().x;
            double dy = waypoints[i].second - raw.back().y;
            double dif = std::hypot(dx, dy); 
            if (dif < 0.01) continue;
            else p.s = raw.back().s + dif;
            raw.push_back(p);
        }
        if (raw.size() < 2) return;
        //Circle and curvature math
        for (int i = 1; i < raw.size() - 1; i++) {
            double a = distance(raw[i - 1], raw[i]);
            double b = distance(raw[i], raw[i + 1]);
            double c = distance(raw[i - 1], raw[i + 1]);
            
            double cross = (raw[i].x - raw[i - 1].x) * (raw[i + 1].y - raw[i].y) - 
                           (raw[i].y - raw[i - 1].y) * (raw[i + 1].x - raw[i].x);
            double denom = (a * b * c);
            if (fabs(denom) <= 1e-9) raw[i].curvature = 0;
            else raw[i].curvature = 2 * cross / denom;            
        }
        if (startReplaced && raw.size() >= 3 && std::fabs(raw[2].curvature) < std::fabs(raw[1].curvature)) 
            raw[1].curvature = raw[2].curvature;
        
        pts.push_back(raw[0]);
        for (int i = 0; i < raw.size() - 1; i++) {
            PathPoint a = raw[i], b = raw[i + 1];
            double dx = b.x - a.x, dy = b.y - a.y;
            double ds = std::hypot(dx, dy);
            int spacing = std::ceil(ds / configs.maxPtSpacing);
            double xSpace = dx / spacing, ySpace = dy / spacing;
            double sSpace = ds / spacing, curvSpace = (b.curvature - a.curvature) / spacing;
            for (int j = 1; j <= spacing; j++) {
                pts.push_back({xSpace * j + a.x, 
                               ySpace * j + a.y, 
                               sSpace * j + a.s, 
                               curvSpace * j + a.curvature});
            }
        }

        //Initializing speed for every point
        for (int i = 0; i < pts.size(); i++) {
            double v = configs.maxSpeed;
            if (fabs(pts[i].curvature) >= 1e-9) v = std::min(v, configs.turnK / std::fabs(pts[i].curvature));
            v = std::min(v, configs.maxSpeed / (1 + std::fabs(pts[i].curvature) * trackWidth / 2));
            pts[i].vel = v;
        }
        // speed_before²  =  speed_after²  +  2 × a × distance
        // a is how efficient the breaking of the robot is, brakeRate is going to be the 2a part isolated
        // The robot's braking isn't linear, instead the stopping distance grows with the square of speed
        pts.back().vel = 0;
        double brakeRate = 127 * 127 / configs.decelDist; // the braking rate from the formula (2 * a)
        for (int i = pts.size() - 2; i >= 0; i--) {
            double d = pts[i + 1].s - pts[i].s;
            pts[i].vel = std::min(pts[i].vel, std::sqrt(std::pow(pts[i + 1].vel, 2) + d * brakeRate)); 
            // the previus' speed squared plus the exponetialy decreasing braking speed multiplied by the distance to the next point
        }
        

    }

public: //Const accepts written in lists like {{0, 0}, {10, 5}}

    Path (const std::vector<PathVec>& waypoints, const PathConfig& configs, double trackWidth) : 
          waypoints(waypoints), configs(configs), trackWidth(trackWidth) { build(); }
        
    void setStart(double x, double y){
        startReplaced = true;
        if (!waypoints.empty()) {
            waypoints[0].first = x;
            waypoints[0].second = y;
            build();
        }
    }
    const PathPoint& operator[](size_t i) const {return pts[i]; }
    int size() const {return pts.size(); }
    bool valid() const {return (pts.size() >= 2); }
    const PathPoint& back() const { return pts.back(); }
    double length() const {return (pts.empty()) ? 0.0 : pts.back().s; }
    const PathConfig& config() const {return configs; }
    double getTrackWidth() const { return trackWidth; }
    
};

class PurePursuit {
private:
    Path path;
    size_t closestIdx = 0; //Closest point, always moves in direction of travel
    double lastLookFrac = 0.0;
    double prevVel = 0.0;
    double lookX = 0.0, lookY = 0.0;
    double prevL = 0.0, prevR = 0.0;

    /*
    Defining two vectors to help us:
        * d = F - E
        * f = E - C
        * Where F is the second point, E is the first point, and C is the robot's position
    spot(t) is the mathematical function for how far along E and F we are
    spot(t) = E + t*(F - E)

    When representing the position relative to the robot
        * v(t) = 
        * spot(t) - C = 
        * (E - C) + t(F - E) =
        * f + t(d)
    When solving for the actual distance we need to square root, but we won't because of complexity
    Instead we will expand the equation into a quadratic equation and solve for it
    L = Lookahead Distance -> what we have been trying to solve for
    L^2 = (f + td) ^ 2 = (f*f) + f(td) + f(td) + (td)*(td)
    Rearanging we get: (d*d) t^2 + 2(f*d) t + (f*f - L^2) = 0
    Which gives us our quadratic equation and variables of: a = d^2, b = 2fd, c = f^2-L^2
    */

    bool findLookahead(double x, double y, double L) {
        if (!path.valid()) return false;
        // updateClosest(x, y, L);

        int startIdx = std::min(
            std::max(
                (int)closestIdx, 
                (int)std::floor(lastLookFrac)),
            path.size() - 2);

        for (int i = startIdx; i <= path.size() - 2; i++) {
            if(path[i].s > 2 * L + path[closestIdx].s) break;

            double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y; //d = F - E
            double fx = path[i].x - x, fy = path[i].y - y; //f = E - C

            double a = dx * dx + dy * dy; // Quadratic formula variables
            double b = 2 * (fx * dx + fy * dy);
            double c = fx * fx + fy * fy - L * L;
            double disc = b*b - 4 * a * c;
            if (a <= 1e-9 || disc < 0) continue;

            double t1 = (-b - std::sqrt(disc)) / (2 * a);
            double t2 = (-b + std::sqrt(disc)) / (2 * a);

            for (double t: {t2, t1}) {
                if (t >= 0 && (t <= 1 || i == path.size() - 2)) {
                    double frac = i + t;
                    if (frac >= lastLookFrac && frac >= closestIdx) {
                        lookX = path[i].x + t * (path[i + 1].x - path[i].x);
                        lookY = path[i].y + t * (path[i + 1].y - path[i].y);
                        lastLookFrac = frac; 
                        return true;
                    }
                }
            }
        }
        return false;
    }

    double arcCurv(Pose& p, bool reversed, bool& behind, double& side) {
        
        double h = deg2rad(p.theta);
        double dx = lookX - p.x, dy = lookY - p.y;
        if (reversed) h += M_PI;
        side = -dx * std::sin(h) + dy * std::cos(h);

        double D2 = dx * dx + dy * dy;
        if (D2 < 1e-9) return 0.0;
        double curv = 2 * side / D2;

        double fwd = dx * std::cos(h) + dy * std::sin(h);
        behind = (D2 >= 1e-9) && (fwd < 0.0);
        return curv;
    }

    // closestIdx is coarse: points are 2–6" apart, so the robot is usually between points. Get the exact distance by projecting the robot onto the segment from point 
    // A = closestIdx − 1 to point B = closestIdx + 1, clamped to the ends of the path:
    // u = (B − A) / |B − A|                         // unit direction along the path
    // along = (robot − A) · u                       // how far past A the robot is
    // along = clamp(along, 0, B.s − A.s)
    // sRobot = A.s + along
    // · is the dot product: x*x' + y*y'. It measures how far the robot is along that direction. 
    // Save sRobot in a member variable (e.g. progressS), because waitUntil() and stuck detection will use it later.
    double targetSpeed(Pose& pose, double minSpeed, double maxStep) {
        int A = closestIdx > 0 ? closestIdx - 1 : 0;
        int B = std::min((int)closestIdx + 1, path.size() - 1);
        double dx = path[B].x - path[A].x;
        double dy = path[B].y - path[A].y;
        double len = std::hypot(dx, dy);
        double ux = dx / len;
        double uy = dy / len;
        
        double rx = pose.x - path[A].x;
        double ry = pose.y - path[A].y;
        

        double along = ux * rx + uy * ry;
        along = std::clamp(along, 0.0, path[B].s - path[A].s);
        double sRobot = path[A].s + along;
        progressS = sRobot;

        double planned = 0.0;
        for(int i = A; i < path.size() - 1; i++){
            if(path[i].s < sRobot && path[i + 1].s < sRobot) continue;

            if (i == path.size() - 2 || i == path.size() - 1) { //Second last  to last point
                planned = path[path.size() - 2].vel;
                break;
            }

            double dVel = path[i + 1].vel - path[i].vel;
            double d = path[i + 1].s - path[i].s;
            planned = path[i].vel + (dVel / d) * (sRobot - path[i].s);
            break;
        }

        double remaining = path.length() - sRobot;
        double endCap = 127 * std::sqrt(remaining / path.config().decelDist);
        double target = std::min(planned, endCap);
        if (minSpeed > 0 && target < minSpeed) target = minSpeed;
        target = slew(target, prevVel, maxStep);
        prevVel = target;
        return target;
        
    }

    WheelTargets wheelSpeeds(double target, double curv, bool reversed, 
                             double dt, bool behind, double side) { // This might just need to call targ instead of passing in
        //Differential Drive
        double left = target * (1 - curv * path.getTrackWidth()/2);
        double right = target * (1 + curv * path.getTrackWidth()/2);
        if(behind) {
            double sgn = (side >= 0) ? 1.0 : -1.0;
            double pv = target;
            left = -sgn * pv; right = sgn * pv;
        }

        if (reversed) {
            double temp = left;
            left = -right; right = -temp;
        }
        double m = std::max(std::fabs(left), std::fabs(right));
        double maxSpeed = path.config().maxSpeed;
        if (m > maxSpeed) {left *= maxSpeed / m; right *= maxSpeed / m; }
        WheelTargets out = {.leftVel = left, .rightVel = right};
        out.leftAccel = (left - prevL) / dt; out.rightAccel = (right - prevR) / dt;
        prevL = left; prevR = right;

        return out;
    }

    bool isDone(const Pose& pose, double endTolerance, double lookaheadDist) const {
        if (progressS <= std::max(path.length() - lookaheadDist, 0.0)) return false; //Check if we are even nearing done - should fix loopy paths
        int n = path.size();
        double dx = path[n - 1].x - path[n - 2].x, dy = path[n - 1].y - path[n - 2].y;
        double len = std::hypot(dx, dy), ux = dx / len, uy = dy / len;
        double along = (pose.x - path[n - 1].x) * ux + (pose.y - path[n - 1].y) * uy;
        // Along would be negative if we are still before the end where as it would be positive after we passed it
        return along >= -endTolerance; //Return if we are past the end
    }


public:
    PurePursuit(const Path& path, double startSpeed, double startL, double startR) : 
                path(path), prevVel(std::min(startSpeed, 0.0)), prevL(startL), prevR(startR) {}

    double progressS = 0.0;

    size_t updateClosest(double x, double y, double lookaheadDist) {
        if (!path.valid()) return 0;
        size_t endIdx = std::min(closestIdx + 25, (size_t)path.size()) - 1;
        double minDist = std::numeric_limits<double>::max();
        size_t minIdx = (size_t)path.size() - 1;
        for (size_t i = closestIdx; i <= endIdx; i++) {
            if (i > closestIdx + 1 && path[closestIdx].s + lookaheadDist < path[i].s) break;
            double dx = path[i].x - x;
            double dy = path[i].y - y;
            double d = dx * dx + dy * dy;
            if (d < minDist) {
                minDist = d;
                minIdx = i;
            }
        }
        closestIdx = minIdx;
        return closestIdx;
    }
    
    size_t closest() const {return closestIdx; }
    
    FollowReturn step(Pose& p, double lookaheadDist, bool reversed, 
                      double minSpeed, double maxStep, double endTolerance, double dt) {
        FollowReturn follow;
        updateClosest(p.x, p.y, lookaheadDist);
        findLookahead(p.x, p.y, lookaheadDist);
        
        bool behind = false; double side = 0.0;
        //part 2 / 3 not working
        double curv = arcCurv(p, reversed, behind, side);
        double target = targetSpeed(p, minSpeed, maxStep);
        follow.wheels = wheelSpeeds(target, curv, reversed, dt, behind, side);
        follow.pivoting = behind;
        follow.done = isDone(p, endTolerance, lookaheadDist);
        double dx = p.x - path[closestIdx].x;
        double dy = p.y - path[closestIdx].y;
        follow.distToPath = std::hypot(dx, dy);
        return follow;
    }
};


} //namespace yogurt