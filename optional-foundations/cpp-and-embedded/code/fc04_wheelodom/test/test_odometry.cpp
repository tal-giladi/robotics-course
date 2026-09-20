// FC.04 - tests, run by ctest. No test framework on purpose: one more dependency would hide
// what CMake is doing. ROS 2 packages use ament_cmake_gtest / gtest here; the CMake wiring is
// the same idea - a second executable that links the same library.

#include "wheelodom/odometry.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int g_failures = 0;

void expect_near(const char * what, double actual, double expected, double tolerance) {
  const bool ok = std::fabs(actual - expected) <= tolerance;
  if (!ok) ++g_failures;
  std::printf("%-46s %-4s actual %.6f, expected %.6f +/- %g\n", what, ok ? "ok" : "FAIL", actual,
              expected, tolerance);
}

}  // namespace

int main() {
  const wheelodom::DriveGeometry karmel;

  // 2*pi*0.045 / 2464 = 0.000114750 m per tick = 0.1147 mm
  expect_near("metres_per_tick(karmel)", wheelodom::metres_per_tick(karmel), 0.000114750, 1e-9);

  expect_near("wrap_angle(3*pi/2)", wheelodom::wrap_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  expect_near("wrap_angle(pi)", wheelodom::wrap_angle(M_PI), M_PI, 1e-12);
  expect_near("wrap_angle(-pi)", wheelodom::wrap_angle(-M_PI), M_PI, 1e-12);

  // One full wheel revolution on both wheels = the wheel circumference, straight ahead.
  const wheelodom::Pose start;
  const wheelodom::Pose straight =
      wheelodom::integrate(start, karmel, karmel.ticks_per_wheel_rev, karmel.ticks_per_wheel_rev);
  expect_near("one wheel revolution: x", straight.x_m, 2.0 * M_PI * 0.045, 1e-9);
  expect_near("one wheel revolution: y", straight.y_m, 0.0, 1e-12);
  expect_near("one wheel revolution: theta", straight.theta_rad, 0.0, 1e-12);

  // Equal and opposite ticks = rotation in place, pose unchanged.
  const wheelodom::Pose spun = wheelodom::integrate(start, karmel, -500, 500);
  expect_near("spin in place: x", spun.x_m, 0.0, 1e-12);
  expect_near("spin in place: y", spun.y_m, 0.0, 1e-12);
  expect_near("spin in place: theta", spun.theta_rad,
              2.0 * 500 * wheelodom::metres_per_tick(karmel) / karmel.wheel_separation_m, 1e-12);

  // A closed circle: integrating a constant turn all the way round returns to the start.
  // Each step turns by (right - left) * metres_per_tick / separation. With a 3-tick difference
  // that is 3.4425 mrad, so 1825 steps come to within 0.6 mrad of a full turn.
  wheelodom::Pose pose;
  const std::int32_t delta = 3;
  const double dtheta_per_step =
      2.0 * delta * wheelodom::metres_per_tick(karmel) / karmel.wheel_separation_m;
  const auto steps = static_cast<std::int32_t>(std::lround(2.0 * M_PI / dtheta_per_step));
  for (std::int32_t i = 0; i < steps; ++i) {
    pose = wheelodom::integrate(pose, karmel, 40 - delta, 40 + delta);
  }
  std::printf("  (circle: %d steps of %.4f mrad, radius %.3f m)\n", steps, dtheta_per_step * 1000.0,
              40.0 * wheelodom::metres_per_tick(karmel) / dtheta_per_step);
  expect_near("closed circle returns to the start: x", pose.x_m, 0.0, 5e-3);
  expect_near("closed circle returns to the start: y", pose.y_m, 0.0, 5e-3);

  std::printf("\n%s\n", g_failures == 0 ? "all tests passed" : "FAILURES");
  return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
