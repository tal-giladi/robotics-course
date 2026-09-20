// FC.04 - an executable that USES the library. It includes only the header; the linker is what
// connects this file's call to integrate() with the machine code in odometry.cpp.o.

#include "wheelodom/odometry.hpp"

#include <cmath>
#include <cstdio>

int main() {
  const wheelodom::DriveGeometry karmel;    // the defaults: 45 mm wheels, 200 mm apart, 2464 ticks
  std::printf("metres per tick: %.6f m  (%.3f mm)\n",
              wheelodom::metres_per_tick(karmel), 1000.0 * wheelodom::metres_per_tick(karmel));

  // Drive 1 m straight, then turn 90 degrees left on the spot, then 0.5 m straight.
  const double mpt = wheelodom::metres_per_tick(karmel);
  const auto ticks_for = [mpt](double metres) {
    return static_cast<std::int32_t>(std::lround(metres / mpt));
  };
  // A pure rotation of dtheta needs each wheel to travel dtheta * separation / 2 in opposite
  // directions.
  const double quarter_turn_m = (M_PI / 2.0) * karmel.wheel_separation_m / 2.0;

  wheelodom::Pose pose;
  std::printf("%-22s x=%7.4f m  y=%7.4f m  theta=%7.2f deg\n", "start", pose.x_m, pose.y_m,
              pose.theta_rad * 180.0 / M_PI);

  pose = wheelodom::integrate(pose, karmel, ticks_for(1.0), ticks_for(1.0));
  std::printf("%-22s x=%7.4f m  y=%7.4f m  theta=%7.2f deg\n", "1 m forward", pose.x_m, pose.y_m,
              pose.theta_rad * 180.0 / M_PI);

  pose = wheelodom::integrate(pose, karmel, -ticks_for(quarter_turn_m), ticks_for(quarter_turn_m));
  std::printf("%-22s x=%7.4f m  y=%7.4f m  theta=%7.2f deg\n", "turn left 90 deg", pose.x_m,
              pose.y_m, pose.theta_rad * 180.0 / M_PI);

  pose = wheelodom::integrate(pose, karmel, ticks_for(0.5), ticks_for(0.5));
  std::printf("%-22s x=%7.4f m  y=%7.4f m  theta=%7.2f deg\n", "0.5 m forward", pose.x_m, pose.y_m,
              pose.theta_rad * 180.0 / M_PI);

  // One control step of a gentle arc, the way karmel_hardware would see it at 100 Hz.
  wheelodom::Pose arc;
  for (int step = 0; step < 100; ++step) {
    arc = wheelodom::integrate(arc, karmel, 20, 24);   // 1 s of a slow left curve
  }
  std::printf("\n100 steps of (20, 24) ticks: x=%.4f m  y=%.4f m  theta=%.2f deg\n",
              arc.x_m, arc.y_m, arc.theta_rad * 180.0 / M_PI);
  return 0;
}
