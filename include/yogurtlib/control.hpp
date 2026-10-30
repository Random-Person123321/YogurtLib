#pragma once

namespace yogurt {
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