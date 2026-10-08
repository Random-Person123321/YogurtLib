// #include "main.h"
// #include <iomanip>
// #include <random>

// // TODO-REMOVE: getCurrentHeading/updateOdometry/resetOdometry/terminalDebug
// // are all superseded by Drivetrain's internal odometry (calibrate/setPose/
// // the private updateOdometry loop in dt.hpp). distanceReset() was already
// // explicitly decided against porting into Drivetrain — MCL will read the
// // raw distance sensors directly instead (see the plan's "Object layering"
// // section) — so it's safe to drop too.
// //
// // TODO-PORT (optional, not urgent): updateHeadingFilter/readGyroZ_dps/
// // resetHeadingFilter/avgHeadingDeg implement a gyro-predict + heading-correct
// // complementary filter that's more sophisticated than Drivetrain's current
// // readHeadingDeg() (which just circular-averages raw IMU headings, no gyro
// // fusion). This filter was already unused/disabled in the old code too
// // (readHeadingDegBest was used instead), so it isn't blocking anything —
// // but it's real, valuable design work worth porting into Drivetrain later
// // as a better heading estimator, not just deleting.

// // state for odom tick deltas (must be reset-safe)
// static double g_lastVertTicks = 0.0;
// static double g_lastHorizTicks = 0.0;
// static double g_lastHeadingDeg = 0.0;

// static inline double inchesPerTick(double wheelDiamIn) {
//   // pros::Rotation get_position() returns centidegrees (0.01 deg)
//   // 36000 "ticks" per revolution.
//   return (wheelDiamIn * M_PI) / 36000.0;
// }

// // =======================
// // Complementary Heading Filter (gyro-predict + heading-correct)
// // =======================
// static bool   g_headInit = false;
// static double g_headRad = 0.0;          // filtered heading state (radians, wrapped)
// static double g_gyroZ_dps = 0.0;        // low-passed gyro z rate (deg/s)
// static bool   g_measInit = false;
// static double g_measRadLP = 0.0;
// static constexpr double MEAS_LP_ALPHA = 0.3; // 0.10–0.25


// // Tunables (rarely need changes)
// static constexpr double GYRO_LP_ALPHA = 0.37;   // 0..1 (higher = more responsive)0.25-0.35
// static constexpr double CORRECT_GAIN  = 0.15;   // 0..1 (higher = trusts heading more) 0.06-0.12

// // If your heading direction is opposite, flip this to -1
// static constexpr double GYRO_SIGN = -1.0; // Higher values for CCW turn

// // Circular average of two headings (unit circle)
// static double avgHeadingDeg(double h1_deg, double h2_deg) {
//   double r1 = deg2rad(h1_deg);
//   double r2 = deg2rad(h2_deg);
//   double s = std::sin(r1) + std::sin(r2);
//   double c = std::cos(r1) + std::cos(r2);
//   return wrapDeg(rad2deg(std::atan2(s, c)));
// }

// // Read heading from both IMUs, reject outlier if they disagree hard
// static double readHeadingDegBest(double currentFilteredDeg) {
//   std::vector<double> rawVal = {};
//   std::vector<bool> bad = {};
//   int badCnt = 0;

//   for (auto* imu : drivetrain.odom.imus.sensors) {
//     double raw = imu->get_heading();
//     //Convert from "compass" to math unit circle units
//     raw = wrapDeg(90 - raw); // Math conversion is: 90 - compassD = mathD
//     rawVal.push_back(raw);
//   }
//   for (double raw : rawVal) {
//     bool temp = std::isnan(raw) || std::isinf(raw);
//     if (temp) badCnt++;
//     bad.push_back(temp);
//   }
//   // if (badCnt == imus.size()) return currentFilteredDeg; //If all imus are bad return last measured value
//   // if (bad1 && bad2) return currentFilteredDeg;
//   // if (bad1) return headingToSignedDeg(raw2);
//   // if (bad2) return headingToSignedDeg(raw1);

//   // double h1 = headingToSignedDeg(raw1);
//   // double h2 = headingToSignedDeg(raw2);

//   // double diff = std::fabs(angleDiffDeg(h1, h2));
//   // if (diff > 25.0) {
//   //   double e1 = std::fabs(angleDiffDeg(h1, currentFilteredDeg));
//   //   double e2 = std::fabs(angleDiffDeg(h2, currentFilteredDeg));
//   //   return (e1 <= e2) ? h1 : h2;
//   // }

//   // return avgHeadingDeg(h1, h2);
//   return 1.0;
// }


// static void resetHeadingFilter(double headingDeg) {
//   g_headInit = true;
//   g_headRad = deg2rad(wrapDeg(headingDeg));
//   g_gyroZ_dps = 0.0;
// }

// double getCurrentHeading() {
//   // If you still want a raw "best" heading without filtering:
//   double current = g_headInit ? wrapDeg(rad2deg(g_headRad)) : 0.0;
//   return readHeadingDegBest(current);
// }

// // void resetOdometry(double startX, double startY, double startHeading) {
// //   pose = {startX, startY, wrapDeg(startHeading)};

// //   double compassHeading = 90 - startHeading;
// //   compassHeading = wrapDeg(compassHeading);
// //   if (compassHeading < 0) compassHeading += 360;
// //   imu1.set_heading(compassHeading);
// //   imu2.set_heading(compassHeading);

// //   g_lastVertTicks = verticalOdom.get_position();
// //   g_lastHorizTicks = horizontalOdom.get_position();
// //   g_lastHeadingDeg = pose.heading;

// //   resetHeadingFilter(pose.heading);
// // }

// void updateOdometry(void* param) {
//   // (void)param;

//   // const double V_IN_PER_TICK = inchesPerTick(VERT_DIAMETER);
//   // const double H_IN_PER_TICK = inchesPerTick(HORIZ_DIAMETER);

//   // resetOdometry(pose.x, pose.y, pose.heading);

//   // const int stepMs = 10; //30 ms?
//   // const double dt = stepMs / 1000.0;

//   // std::random_device rd;
    
//   //   // 2. Initialize the standard Mersenne Twister engine with the seed
//   // std::mt19937 gen(rd());
    
//   // // 3. Define the distribution range [inclusive, inclusive]
//   // // std::uniform_int_distribution<> distr(1, 100); 
//   // std::uniform_real_distribution<double> double_distr(-1.5, 1.5); // For decimals between 0.0 and 1.0


//   // // 4. Generate a random number
//   // double random_head = double_distr(gen);

//   // while (true) {
//   //   double vNow = verticalOdom.get_position();
//   //   double hNow = horizontalOdom.get_position();
//   //   if (std::isnan(vNow) || std::isinf(vNow)) vNow = 0.0;
//   //   if (std::isnan(hNow) || std::isinf(hNow)) hNow = 0.0;

//   //   // double headDeg = updateHeadingFilter(dt);
//   //   double headDeg = wrapDeg(getCurrentHeading());

//   //   double dHeadDeg = angleDiffDeg(headDeg, g_lastHeadingDeg) + double_distr(gen); //Adds random heading motion to particles
//   //   double dTheta = deg2rad(dHeadDeg);

//   //   double dV = (vNow - g_lastVertTicks) * V_IN_PER_TICK;
//   //   double dH = (hNow - g_lastHorizTicks) * H_IN_PER_TICK;

//   //   double forward = dV - (VERT_RIGHT_OFFSET * dTheta);
//   //   double left = dH - (HORIZ_FWD_OFFSET * dTheta);

//   //   double mid = deg2rad(g_lastHeadingDeg) + dTheta * 0.5;
//   //   double cosT = std::cos(mid);
//   //   double sinT = std::sin(mid);


//   //   //MCL Particle updates:
//   //   for(int i = 0; i < particle_Num; i++){
//   //     double particlex = particles[i].x;
//   //     double particley = particles[i].y;
//   //     double particleh = particles[i].h;
//   //     particlex += forward * cosT - left * sinT;
//   //     particley += forward * sinT + left * cosT;
//   //     //Need to add noise + heading
      

//   //     //Update particle with movement
//   //     particles[i].x = particlex;
//   //     particles[i].y = particley;
//   //     particles[i].h = particleh;
//   //   }


//   //   pose.heading = headDeg;

//   //   g_lastVertTicks = vNow;
//   //   g_lastHorizTicks = hNow;
//   //   g_lastHeadingDeg = headDeg;

//   //   pros::delay(stepMs);
//   // }
// }

// // Distance relocalization (BLENDED, not teleport)
// void distanceReset(pros::Distance& sensor,
//                    double knownFeaturePos,
//                    double sensorToCenterOffset, //(+)on measuring side
//                    bool isX, //Sensor on X or Y axis
//                    bool sensorDirPositive, //+x East, +y North
//                    double angleOffset) { //The angle offset the sensor is mounted at (Pointint up = (+) vice verca)
// //   auto readInches = [&]() -> double {
// //     int mm = sensor.get();
// //     if (mm == PROS_ERR) return NAN;
// //     return mm / 25.4;
// //   };

// //   angleOffset = std::abs(angleOffset);
// //   angleOffset = deg2rad(angleOffset);

// //   double a = readInches(); pros::delay(34);
// //   double b = readInches(); pros::delay(34);
// //   double c = readInches();

// //   if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c)) return;

// //   double reading = (a + b + c) / 3.0;
// //   if (reading < 1.5 || reading > 78.0) return;
// //   // Adjust reading based on the angle offset the sensor is mounted at
// //   reading = std::cos(angleOffset) * reading;

// //   if (std::fabs(a - b) > 2.0 || std::fabs(b - c) > 2.0 || std::fabs(a - c) > 2.0) return;

// //   double sign = sensorDirPositive ? -1.0 : 1.0;
// //   double corrected = knownFeaturePos + sign * (reading + sensorToCenterOffset);

// //   constexpr double alpha = 0.6; //0.35
// //   if (isX) pose.x += (corrected - pose.x) * alpha;
// //   else pose.y += (corrected - pose.y) * alpha;
// // }

// // void terminalDebug() {
// //   int cnt = 0; //0 for temp, 1 for voltages, 2 for imus
// //   while (true) {
// //     // Snapshot pose (avoid partial update mid-print)
// //     // RobotPose p = pose;
    
// //     //Temperature and motor voltages prints to the controller
// //     // if (pros::millis() - time >= 200) {
// //     //   }
// //     //   volt = !volt;
// //     //   time = pros::millis();
// //     // }
// //     // switch(cnt) {
// //     //   case 0: temps(); master.clear_line(1); cnt++;
// //     //   case 1: voltages(); master.clear_line(2); cnt++;
// //     //   case 2: inertials(); master.clear_line(0); cnt = 0;
// //     // }
// //     if (cnt == 0) {
// //       temps(); master.clear_line(1); cnt++;
// //     }
// //     else if (cnt == 1) {
// //       // voltages(); master.clear_line(2); cnt++;
// //       printPose(); master.clear_line(2); cnt++;
// //     }
// //     else {
// //       inertials(); master.clear_line(0); cnt = 0;
// //     }
// //     // pros::lcd::print(0, "X: %.1f", p.x);
// //     // pros::lcd::print(1, "Y: %.1f", p.y);
// //     // pros::lcd::print(2, "H: %.1f", p.heading);
// //     // pros::lcd::print(3, "IMU1: %.0f", imu1.get_heading());
// //     // pros::lcd::print(4, "IMU2: %.0f", imu2.get_heading());
// //     /*
// //     std::vector<double> left_drive = left_mg.get_temperature_all();
// //     std::vector<double> right_drive = right_mg.get_temperature_all();
// //     for (auto i : left_drive) {
// //       std::cout << std::setprecision(1) << i << " ";
// //     }
// //     for (auto i : right_drive) {
// //         std::cout << std::fixed << std::setprecision(1) << i << " ";
// //     }
// //     */
// //     printf("(X: %.1lf, Y: %.1lf, H: %.1lf, MU1: %.1lf, MU2: %.1lf)", p.x, p.y, p.heading, imu1.get_heading(), imu2.get_heading());
// //     std::cout << " Time: "<< (pros::millis() - startTime) / 1000 << "s\n";
// //     pros::delay(200); // 5 Hz (safe)

// //   }
// }
