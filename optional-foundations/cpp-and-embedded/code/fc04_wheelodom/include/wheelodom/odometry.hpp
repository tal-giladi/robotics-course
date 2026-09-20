// FC.04 - the HEADER: what exists. Everything that includes this file learns these names.
//
// Include guard: without it, a file that includes this header twice (directly and through
// another header) would declare Pose twice and the compiler would reject the translation unit.
// #pragma once does the same thing in one line and every compiler this course uses supports it,
// but the explicit guard is what you will see in ROS 2 headers, so learn to read it.
#ifndef WHEELODOM_ODOMETRY_HPP_
#define WHEELODOM_ODOMETRY_HPP_

#include <cstdint>

namespace wheelodom {

/// Robot pose in the odom frame. REP-103: x forward, y left, theta counter-clockwise, SI units.
struct Pose {
  double x_m = 0.0;
  double y_m = 0.0;
  double theta_rad = 0.0;
};

/// The physical constants of one differential-drive robot. Defaults are karmel
/// (labs/config/karmel.yaml): 45 mm wheels, 200 mm apart, 2464 ticks per wheel revolution.
struct DriveGeometry {
  double wheel_radius_m = 0.045;
  double wheel_separation_m = 0.200;
  std::int32_t ticks_per_wheel_rev = 2464;
};

/// Metres travelled by one wheel for one encoder tick.
/// Small enough to be defined here: the compiler can inline it into every caller.
double metres_per_tick(const DriveGeometry & geometry);

/// Wrap an angle into (-pi, pi].
double wrap_angle(double angle_rad);

/// Integrate one encoder update into the pose (exact arc model, not the small-angle version).
/// `left_delta_ticks` / `right_delta_ticks` are the *change* since the previous call.
Pose integrate(const Pose & pose, const DriveGeometry & geometry,
               std::int32_t left_delta_ticks, std::int32_t right_delta_ticks);

}  // namespace wheelodom

#endif  // WHEELODOM_ODOMETRY_HPP_
