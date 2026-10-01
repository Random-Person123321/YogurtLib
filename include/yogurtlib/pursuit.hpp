#pragma once
#include "main.h"

namespace yogurt{

struct PathPoint {
    double x = 0.0, y = 0.0;
    double s = 0.0; // How many inches along the path the robot is
    double curvature = 0.0, vel = 0.0; // Curvature and velocity (0-127)
};

struct PathConfig {
    double maxSpeed = 127.0; //max speed the robot drives, out of 127
    double decelDist = 24.0; //Inches that allows the robot to slow from full speed to 0. Smaller number means braking later
    double slew = 600.0; //mV per loop. How much to speed up while accelerating to reduce wheel slip
    double turnK = 5.0; //speed(0-127) per inch of turn radius. How much to slow down by during tight curves/turns
};

class Path {
private:
    std::vector<std::pair<double, double>> waypoints;
    std::vector<PathPoint> pts;
    PathConfig configs;
    double trackWidth = 0.0; // inches, center-to-center of left/right wheels

    double distance (PathPoint& first, PathPoint& second) {
        double dx = second.x - first.x;
        double dy = second.y - first.y;
        return std::hypot(dx, dy);

    }
    void build() {
        pts.clear();
        for (size_t i = 0; i < waypoints.size(); i++){
            if (pts.empty()) pts.push_back({});
            double x, y;
            x = waypoints[i].first;
            y = waypoints[i].second;
            pts.push_back({x, y});

            double dx = waypoints[i].first - pts.back().x;
            double dy = waypoints[i].second - pts.back().y;
            double dif = std::hypot(dx, dy);
            if (dif < 0.01) continue;
            else pts[i].s = pts[i - 1].s + dif;
        }
        //Circle and curvature math
        for (int i = 1; i < pts.size() - 1; i++){
            double a = distance(pts[i - 1], pts[i]);
            double b = distance(pts[i], pts[i + 1]);
            double c = distance(pts[i - 1], pts[i + 1]);
            
            double cross = (pts[i].x - pts[i - 1].x) * (pts[i + 1].y - pts[i].y) - 
                           (pts[i].y - pts[i - 1].y) * (pts[i + 1].x - pts[i - 1].x);
            pts[i].curvature = 2 * cross / (a * b * c);
        }
    }

public: //Const accepts written in lists like {{0, 0}, {10, 5}}
    Path (const std::vector<std::pair<double, double>>& waypoints, const PathConfig& configs, double trackWidth) : 
          waypoints(waypoints), configs(configs), trackWidth(trackWidth) { build(); }
        
    void setStart(double x, double y){
        if (!waypoints.empty()) {
            waypoints[0].first = x;
            waypoints[0].second = y;
            build();
        }
    }
    const PathPoint& operator[](size_t i) const {return pts[i]; }
    int size() const {return pts.size(); }
    bool valid() const {return (pts.size() >= 2); }
    const PathPoint back() const { return pts.back(); }
    double length() const {return (pts.empty()) ? 0.0 : pts.back().s; }
    const PathConfig& config() const {return configs; }
    double getTrackWidth() const { return trackWidth; }
    
};

} //namespace yogurt