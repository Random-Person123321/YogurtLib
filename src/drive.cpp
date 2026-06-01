#include "main.h"



// ---------- DRIVE STRAIGHT (field-consistent) ----------
int driveStraight(double distanceInches,
                  int timeoutMs,
                  double headingOffset,
                  double maxVoltage,
                  double slewRate,
                  double minVoltage) {

  // Outputs are in mV. These are safe starter values; tune kP first.
  yogurt::PID distPID(525.0, 0.0, 33.0, 3.0);     // mV per inch error-ish
  yogurt::PID headPID(300.0, 0.0, 5.0, 2.0);     // mV per degree error-ish

  distPID.reset();
  headPID.reset();

  double startX = pose.x; 
  double startY = pose.y;
  double startHeading = wrapDeg(pose.heading + headingOffset);
  double minV = -maxVoltage, maxV = maxVoltage;
  if (minVoltage != 0.0) {
    if (distanceInches < 0.0) {
      maxV = std::copysignf(minVoltage, distanceInches);
    }
    else {
      minV = std::abs(minVoltage);
    }
  }

  double targetX = startX + distanceInches * std::cos(deg2rad(startHeading));
  double targetY = startY + distanceInches * std::sin(deg2rad(startHeading));

  double prevL = 0.0, prevR = 0.0;

  yogurt::Settle settle;
  settle.errTol = 0.5;     // inches
  settle.velTol = 1.0;      // inches/s
  settle.settleMs = 250;

  int elapsed = 0;
  const int stepMs = 10;
  const double dt = stepMs / 1000.0;

  while (elapsed < timeoutMs) {
    double dx = targetX - pose.x;
    double dy = targetY - pose.y;

    // Signed distance along the intended heading line (prevents weird sign flips)
    double cosH = std::cos(deg2rad(startHeading));
    double sinH = std::sin(deg2rad(startHeading));
    double distErr = dx * cosH + dy * sinH;

    double headErr = angleDiffDeg(startHeading, pose.heading);

    // Settle check on distance error
    if (settle.update(distErr, dt)) break;

    // PID outputs in mV
    double driveOut = distPID.calculateError(distErr, dt);
    double turnOut  = headPID.calculateError(headErr, dt);
    driveOut = clampd(driveOut, minV, maxV);

    // Small static friction feedforward helps consistency
    const double kS = 900.0; // mV (tune 600–1400)
    if (std::fabs(distErr) > 1.0) driveOut += std::copysign(kS, distErr);

    double leftV  = driveOut - turnOut;
    double rightV = driveOut + turnOut;

    // Clamp and battery scale
    double scale = batteryScale();
    leftV  = clampd(leftV * scale,  minV, maxV);
    rightV = clampd(rightV * scale, minV, maxV);

    // Slew (mV per 10ms)
    leftV  = slew(leftV,  prevL, slewRate);
    rightV = slew(rightV, prevR, slewRate);
    leftV  = clampd(leftV,  minV, maxV);
    rightV = clampd(rightV, minV, maxV);

    prevL = leftV;
    prevR = rightV;

    left_mg.move_voltage((int)leftV);
    right_mg.move_voltage((int)rightV);

    pros::delay(stepMs);
    elapsed += stepMs;
  }
  left_mg.move_voltage(0);
  right_mg.move_voltage(0);
  left_mg.brake();
  right_mg.brake();
  return elapsed;
}

// ---------- TURN TO ANGLE (wrap-safe + settle) ----------
void turnToAngle(double targetAngle,
                 int timeoutMs,
                 double maxVoltage,
                 double slewRate) {

  yogurt::PID turnPID(120.0, 0.0, 15.35, 2.0);  // mV/deg //11 //120kp
  //260kp
  turnPID.reset();

  yogurt::Settle settle;
  settle.errTol = 0.75;     // degrees
  settle.velTol = 8.0;     // 8 deg/s
  settle.settleMs = 200;

  double prevV = 0.0;
  int elapsed = 0;
  const int stepMs = 10;
  const double dt = stepMs / 1000.0;

  while (elapsed < timeoutMs) {
    double err = angleDiffDeg(targetAngle, pose.heading);
    // Deadband: ignore tiny error jitter so we don't hunt forever
    if (std::fabs(err) < 0.5) err = 0.0;
    if (settle.update(err, dt)) break;

    double out = turnPID.calculateError(err, dt);

    // Static friction feedforward improves repeatability at low error
    const double kS = 1100.0; // mV, tune 800–1600
    if (std::fabs(err) > 1.5) out += std::copysign(kS, err);

    double scale = batteryScale();
    double v = clampd(out * scale, -maxVoltage, maxVoltage);

    // Slew
    v = slew(v, prevV, slewRate);
    v = clampd(v, -maxVoltage, maxVoltage);
    prevV = v;

    left_mg.move_voltage((int)-v);
    right_mg.move_voltage((int)v);

    pros::delay(stepMs);
    elapsed += stepMs;
  }
  left_mg.move_voltage(0);
  right_mg.move_voltage(0);
  left_mg.brake();
  right_mg.brake();
}

// Circle-segment intersection. Returns the intersection further along segment (larger t).
static bool circleSegmentIntersection(
    double cx, double cy, double r,
    double x1, double y1, double x2, double y2,
    double &ix, double &iy, double &t_out
) {
  double dx = x2 - x1;
  double dy = y2 - y1;

  double fx = x1 - cx;
  double fy = y1 - cy;

  double a = dx * dx + dy * dy;
  if (a < 1e-9) return false;

  double b = 2.0 * (fx * dx + fy * dy);
  double c = fx * fx + fy * fy - r * r;

  double disc = b * b - 4 * a * c;
  if (disc < 0) return false;

  disc = std::sqrt(disc);
  double t1 = (-b - disc) / (2 * a);
  double t2 = (-b + disc) / (2 * a);

  bool ok1 = (t1 >= 0.0 && t1 <= 1.0);
  bool ok2 = (t2 >= 0.0 && t2 <= 1.0);
  if (!ok1 && !ok2) return false;

  double t = -1.0;
  if (ok1 && ok2) t = std::max(t1, t2);
  else if (ok1)   t = t1;
  else            t = t2;

  ix = x1 + t * dx;
  iy = y1 + t * dy;
  t_out = t;
  return true;
}

// Find lookahead point by intersecting circle with path segments,
//choosing the intercection furthest along the segment
// and rejecting intersections "behind" the robot (xR < -behindTol).
static bool findLookaheadPoint(
    const std::vector<std::pair<double,double>> &path,
    double lookaheadDist,
    size_t &lastSeg,
    double nHeading,
    double forwardTol,  // inches (tolerance for points around robot ~ -2in behind)
    double &lookX,
    double &lookY
) {
  const double cx = pose.x, cy = pose.y, r = lookaheadDist;

  bool found = false;
  size_t bestSeg = 0;
  double bestT = -1.0;
  size_t closest = lastSeg;
  double bestD = 1e9;

  size_t start = (lastSeg > 15) ? lastSeg - 15 : 0; // Go 15 points ago or start of array
  size_t end   = std::min(path.size(), lastSeg + 25); // end of array or check 25 after in case off track

  for (size_t i = start; i < end; ++i) {
    double dx = path[i].first - cx;
    double dy = path[i].second - cy;
    double d = dx*dx + dy*dy;
    if (d < bestD) {
      bestD = d; closest = i; // Record down shortest dist to nearest path
    }
  }

  size_t index = std::min(closest + 1, path.size() - 1);
  double bestX = path[index].first;
  double bestY = path[index].second;


  auto findFurthestPoint = [&](size_t i, double ix, double iy, double t) {
    // Transform candidate into robot frame (+x forward)
    double dx = ix - cx;
    double dy = iy - cy;
    double h  = deg2rad(nHeading);
    double cosT = std::cos(h);
    double sinT = std::sin(h);
    double xR = dx * cosT + dy * sinT;

    if (xR < forwardTol) return;

    if (!found || i > bestSeg || (i == bestSeg && t > bestT)) {
      found = true;
      bestSeg = i;
      bestT = t;
      bestX = ix;
      bestY = iy;
    }
  };

  auto scan = [&](size_t start, size_t end) {
    for (size_t i = start; i < end; ++i) {
      double x1 = path[i].first,     y1 = path[i].second;
      double x2 = path[i+1].first,   y2 = path[i+1].second;

      double ix, iy, t;
      if (!circleSegmentIntersection(cx, cy, r, x1, y1, x2, y2, ix, iy, t)) continue;

      findFurthestPoint(i, ix, iy, t);
    }
  };

  // normal scan from lastSeg forward
  if (path.size() >= 2) {
    scan(lastSeg, path.size() - 1);
    // fall back to stop quite mid run - if nothing found, reacquire by scanning from start
    if (!found && lastSeg != 0) {
      scan(0, lastSeg);
    }
  }

  lookX = bestX;
  lookY = bestY;

  if (found) {
    lastSeg = bestSeg;
    return true;
  }
  else lastSeg = closest;
  //Nothing found, target end point
  return false;
}

// Actual Pure Pursuit - uses circle and line segment intersection
//Could add another enumeration to report results when coding autons ->
// Helps if pure pursuit fails mid but needs to go to a position to recover/continue
/*
enum class FollowResult {
  ReachedEnd,
  Timeout,
  NoIntersection_TargetBehind,
  Stuck
};
*/
FollowResult followPath(const std::vector<std::pair<double, double>>& path,
                double lookaheadDist, // inches — tune 10–18
                double maxVoltage, // mV
                int timeoutMs,
                double slewRate, //mv per 10ms
                PursuitDir dir,
                bool progressCheck)
{
  if (path.size() < 2) return FollowResult::PathTooShort;

  const int startTime = pros::millis();
  size_t lastSeg = 0;
  double prevL = 0.0, prevR = 0.0;

  // Tunables (Left internal for now)
  const double stopDist       = 2.0;    // inches to end point to stop
  const double endSlowDist    = 24.0;   // start slowing when within this of end(18.0)
  const double minBase        = 2200.0; // minimum base speed(3k)
  const double curvClampK      = 0.98;  // clamp k = curvature*(track/2) to avoid wheel reversal // 0.90
  const double curvSlowK      = 1.1;    // how hard we slow for curvature(TUNE)
  const double endSlowMinFrac = 0.35;   // min fraction of max near end
  const double forwardTol      = 0.5;   // accepts points more than 0.5" in front
  const double Ld = std::max(lookaheadDist, 1.0); // Accepts no lookahead distance lower than 1"

  FollowResult result = FollowResult::Timeout;
  bool reverse = false;

  double lastEndDist = 1e9;

  int noProgressMs = 0;
  const double progressTol = 0.5; // 0.25 inches
  const int stuckMs = 500;
  int noLookMs = 0;
  const int noLookFailMs = 800;

  if (dir == PursuitDir::Reverse) reverse = true;
  else if (dir == PursuitDir::Forward) reverse = false;
  else { // Auto
    // For Auto direction: decide based on where the end point is in robot frame
    double dxEnd = path.back().first - pose.x;
    double dyEnd = path.back().second - pose.y;
    double h = deg2rad(pose.heading);
    double xR_end = dxEnd * std::cos(h) + dyEnd * std::sin(h); // forward component
    reverse = (xR_end < -2.0); // only reverse if clearly behind
  }

  while (pros::millis() - startTime < timeoutMs) {
    double endDist = std::hypot(path.back().first - pose.x, path.back().second - pose.y);
    if (endDist < stopDist) {
      result = FollowResult::ReachedEnd;
      break;
    }

    if (progressCheck) {
      if (endDist > lastEndDist - progressTol) {
        noProgressMs += 10;
      }
      else {
        lastEndDist = endDist;
        noProgressMs = 0;
      }

      if (noProgressMs >= stuckMs) {
        result= FollowResult::Stuck;
        break;
      }
    }

    // Calculate a new heading: if reversing, treat the robot as facing backwards for pursuit math
    double nHeading = reverse ? wrapDeg(pose.heading + 180.0) : pose.heading;

    //Lookahead point
    double lookX, lookY;
    bool hasLook = findLookaheadPoint(path, Ld, lastSeg, nHeading, forwardTol, lookX, lookY);

    if (!hasLook) noLookMs += 10;
    else noLookMs = 0;

    //Transform to robot frame (+x forward, +y left)
    double dx = lookX - pose.x;
    double dy = lookY - pose.y;

    double h = deg2rad(nHeading);
    double cosT = std::cos(h);
    double sinT = std::sin(h);

    double xR =  dx * cosT + dy * sinT; //forward calculation
    double yR = -dx * sinT + dy * cosT; //left of robot locally
    if (xR < 1.0) xR = 1.0;

    // If can't find a interesection, aiming at end, if also target point behind us - exit
    if(!hasLook && xR < forwardTol) {
      // xR = forwardTol;
      if (noLookMs >= noLookFailMs) {
        size_t closest = lastSeg;
        double bestD = 1e9;

        size_t start = (lastSeg > 15) ? lastSeg - 15 : 0; // Go 15 points ago or start of array
        size_t end = std::min(path.size(), lastSeg + 25); // end of array or check 25 after in case off track

        for (size_t i = start; i < end; ++i) {
          double dx = path[i].first - pose.x;
          double dy = path[i].second - pose.y;
          double d = dx*dx + dy*dy;
          if (d < bestD) {
            bestD = d; closest = i; // Record down shortest dist to nearest path
          }
        }
        driveToPoint(path[closest].first, path[closest].second, 11000.0, false);

        lastSeg = std::max(0, static_cast<int>(closest) - 1);
        // result = FollowResult::NoIntersection_TargetBehind;
        // break;
      }
    }

    //Curvature (positive yR -> left -> positive curvature -> CCW)
    double L = std::hypot(xR, yR);
    double denom = hasLook ? (Ld * Ld) : std::max(1.0, (L * L));
    double curvature = (2.0 * yR) / denom;
    double k = curvature * (TRACK_WIDTH / 2.0);
    k = clampd(k, -curvClampK, curvClampK);

    // Base speed profile: slow near end + slow on tight curvature
    double base = maxVoltage;

    if (endDist < endSlowDist) {
      double frac = endSlowMinFrac + (1.0 - endSlowMinFrac) * (endDist / endSlowDist);
      base = maxVoltage * clampd(frac, endSlowMinFrac, 1.0);
    }

    double curveFrac = 1.0 - std::min(0.70, std::fabs(k) * curvSlowK);
    base *= curveFrac;
    base = clampd(base, minBase, maxVoltage);

    //if reversing, flip both drive to negative (curvature is going to be correct because of new heading(+180))
    double sign = reverse ? -1.0 : 1.0;

    //Differential mapping
    double leftV  = base * (1.0 - sign * k);
    double rightV = base * (1.0 + sign * k);

    leftV *= sign;
    rightV *= sign;

    //Battery scale + slew + clamp
    double scale = batteryScale();
    leftV  = clampd(leftV  * scale, -maxVoltage, maxVoltage);
    rightV = clampd(rightV * scale, -maxVoltage, maxVoltage);

    leftV  = slew(leftV,  prevL, slewRate);
    rightV = slew(rightV, prevR, slewRate);

    leftV  = clampd(leftV,  -maxVoltage, maxVoltage);
    rightV = clampd(rightV, -maxVoltage, maxVoltage);

    prevL = leftV;
    prevR = rightV;

    // If +voltage drives forward on each side, leave as-is:
    left_mg.move_voltage((int)leftV);
    right_mg.move_voltage((int)rightV);

    pros::delay(10);
  }

  left_mg.move_voltage(0);
  right_mg.move_voltage(0);
  left_mg.brake();
  right_mg.brake();
  return result;
}

FollowResult driveToPoint(double targetX, double targetY, double maxVoltage, bool usePurePursuit, bool backwards, int turnTimeout, int driveTimeout) {
  if (usePurePursuit) {
    std::vector<std::pair<double, double>> path = {{pose.x, pose.y}, {targetX, targetY}};
    PursuitDir dir;
    if (backwards) dir = PursuitDir::Reverse;
    else dir = PursuitDir::Forward;
    return followPath(path, 11.5, maxVoltage, 1100, 700.0, dir, false);
  } else {
    RobotPose p = pose;
    double dx = targetX - p.x;
    double dy = targetY - p.y;
    double targetHeading = rad2deg(std::atan2(dy, dx));
    int sign = 1;
    if (backwards) {
      targetHeading = wrapDeg(targetHeading + 180.0);
      sign = -1;

    }
    turnToAngle(targetHeading, turnTimeout);
    p = pose;
    dx = targetX - p.x;
    dy = targetY - p.y;
    driveStraight(sign * std::hypot(dx, dy), driveTimeout, 0.0, maxVoltage);
    return FollowResult::ReachedEnd;
  }
}

FollowResult driveToPose(double targetX, double targetY, double targetHeading, double maxVoltage, bool usePurePursuit, bool backwards, int turnTimeout, int driveTimeout) {
  FollowResult result;
  result = driveToPoint(targetX, targetY, maxVoltage, usePurePursuit, backwards, turnTimeout, driveTimeout);
  turnToAngle(targetHeading, turnTimeout);
  return result;
}

void turnToPoint(double targetX, double targetY, int timeout, double maxVoltage, bool back) {
  double dx = targetX - pose.x;
  double dy = targetY - pose.y;
  double targetHeading = rad2deg(std::atan2(dy, dx));
  if (back) {
    targetHeading = wrapDeg(targetHeading + 180.0);
  }
  turnToAngle(targetHeading, timeout, maxVoltage);
}

void swingToHeading(double targetHeading, bool leftSidePivot, int timeoutMs, double maxVoltage, double slewRate) {
  yogurt::PID swingPID(200.0, 0.0, 16.0, 2.0);
  swingPID.reset();

  int time = 0;
  const int stepMs = 10;
  const double dt = stepMs / 1000.0;
  double prevV = 0.0;

  while (time < timeoutMs) {
    double err = angleDiffDeg(targetHeading, pose.heading);
    if (std::fabs(err) < 1.0) break;

    double out = swingPID.calculateError(err, dt);
    const double kS = 900.0;
    if (std::fabs(err) > 2.0) out += std::copysign(kS, err);

    double scale = batteryScale();
    double v = clampd(out * scale, -maxVoltage, maxVoltage);
    v = slew(v, prevV, slewRate);
    prevV = v;

    if (leftSidePivot) {
      right_mg.move_voltage(0);
      left_mg.move_voltage((int)-v);
    } else {
      left_mg.move_voltage(0);
      right_mg.move_voltage((int)v);
    }

    pros::delay(stepMs);
    time += stepMs;
  }

  left_mg.brake();
  right_mg.brake();
}

