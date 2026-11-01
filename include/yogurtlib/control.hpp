#pragma once

namespace yogurt {

/**
 * @brief The basic's behind all yogurtlib motions, bundels the PID and SETTLE params together
 *
 * @param kP mV per unit of error within a 10 ms timespan
 * @param kI mV based on accumulated error over time; Only use if you dseire to hold a position for long periods of time
 * @param kD mV per unit of error to counteract kP over time
 * @param maxIntergral the maximum windup of kI in mV
 * @param errTol the tolerance in movement unit (Inches or degrees) at which the robot can settle
 * @param velTol how fast the robot can be moving for the robot to settle
 * @param settleMs how long errTol and velTol must be met for the robot to settle 
 *
*/
struct ControllerSettings {
    double kp, ki, kd, maxIntegral;
    double errTol, velTol;
    int settleMs;

    ControllerSettings(double p, double i, double d, double maxInt,
                        double errTol, double velTol, int settleMs);
    
};


// ---- PID (uses dt, proven to work) ----
class PID {
private:
    double kP;
    double kI;
    double kD;

    double maxIntegral;   // clamp on integral term magnitude (0 disables clamp)
    double integral = 0.0;
    double prevError = 0.0;
    bool first = true;

public:
    PID(double p, double i, double d, double maxInt);

    void reset();
    
    // Best method: feed error directly
    double calculateError(double error, double dt);

    // Standard method: target/current
    double calculate(double target, double current, double dt);
};

struct Settle {
  double errTol;
  double velTol;
  int settleMs;

  double lastErr = 1e9;
  int goodMs = 0;

  void reset();

  bool update(double err, double dt);
};

}//yogurt namespace