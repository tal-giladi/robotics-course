// FC.04 - the SOURCE: what it does. This file alone becomes one object file, odometry.cpp.o.
// Change only this file and CMake rebuilds only this file; change the header and everything
// that includes it is rebuilt.

#include "wheelodom/odometry.hpp"

#include <cmath>

namespace wheelodom {

namespace {
// An anonymous namespace = internal linkage = `static` in C, `private` at file scope.
// This name does not exist outside this translation unit, so another .cpp can define its own
// `kTwoPi` without a "multiple definition" link error.
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kStraightThreshold = 1e-9;
}  // namespace

double metres_per_tick(const DriveGeometry & geometry) {
  return kTwoPi * geometry.wheel_radius_m / static_cast<double>(geometry.ticks_per_wheel_rev);
}

double wrap_angle(double angle_rad) {
  double wrapped = std::fmod(angle_rad + M_PI, kTwoPi);
  if (wrapped <= 0.0) wrapped += kTwoPi;
  return wrapped - M_PI;
}

Pose integrate(const Pose & pose, const DriveGeometry & geometry,
               std::int32_t left_delta_ticks, std::int32_t right_delta_ticks) {
  const double scale = metres_per_tick(geometry);
  const double left_m = static_cast<double>(left_delta_ticks) * scale;
  const double right_m = static_cast<double>(right_delta_ticks) * scale;

  const double distance_m = 0.5 * (left_m + right_m);
  const double dtheta = (right_m - left_m) / geometry.wheel_separation_m;

  Pose next;
  if (std::fabs(dtheta) < kStraightThreshold) {
    // Straight line: the arc formula divides by dtheta, so handle this case separately.
    next.x_m = pose.x_m + distance_m * std::cos(pose.theta_rad);
    next.y_m = pose.y_m + distance_m * std::sin(pose.theta_rad);
  } else {
    // Exact arc of radius R = distance / dtheta around the instantaneous centre.
    const double radius = distance_m / dtheta;
    next.x_m = pose.x_m + radius * (std::sin(pose.theta_rad + dtheta) - std::sin(pose.theta_rad));
    next.y_m = pose.y_m - radius * (std::cos(pose.theta_rad + dtheta) - std::cos(pose.theta_rad));
  }
  next.theta_rad = wrap_angle(pose.theta_rad + dtheta);
  return next;
}

}  // namespace wheelodom
