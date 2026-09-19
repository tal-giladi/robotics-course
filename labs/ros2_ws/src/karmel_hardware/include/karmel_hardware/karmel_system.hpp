// karmel_hardware/KarmelSystem — a ros2_control hardware component for the karmel robot.
//
// It is the bridge between ros2_control's world of *interfaces* (doubles in rad and rad/s) and the
// Pico's world of *protocol v1 lines* (integers in ticks and milliradians/s):
//
//     diff_drive_controller ──writes──▶ left_wheel_joint/velocity  ──write()──▶ "V <seq> <l> <r>"
//     diff_drive_controller ◀──reads── left_wheel_joint/position  ◀──read()─── "T <ms> <lticks> ..."
//                                       left_wheel_joint/velocity
//
// Lifecycle (called by the controller_manager's resource manager):
//
//   on_init       parse <ros2_control> parameters from the URDF, check the joints. No I/O.
//   on_configure  open the serial port, say hello, set the Pico's watchdog.     → INACTIVE
//   on_activate   start from zero velocity commands; states are live.           → ACTIVE
//   read/write    every control cycle (controller_manager update_rate).
//   on_deactivate stop the motors ("S").                                        → INACTIVE
//   on_cleanup    close the serial port.                                        → UNCONFIGURED
//
// URDF parameters (<hardware><param name="...">):
//   device                serial device, e.g. /dev/ttyACM0            (required)
//   baud                  default 115200
//   ticks_per_rev         encoder ticks per wheel revolution           (required)
//   watchdog_ms           Pico command watchdog, default 300
//   telemetry_timeout_ms  read() fails if no telemetry for this long, default 500
#ifndef KARMEL_HARDWARE__KARMEL_SYSTEM_HPP_
#define KARMEL_HARDWARE__KARMEL_SYSTEM_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "karmel_hardware/serial_port.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace karmel_hardware
{

class KarmelSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  using return_type = hardware_interface::return_type;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kLeft = 0;
  static constexpr size_t kRight = 1;

  uint32_t next_seq();
  /// Read the port and handle every complete line. False on serial I/O error.
  bool poll_serial();
  void handle_line(const std::string & line);

  // configuration (from the URDF)
  std::string device_;
  int baud_ = 115200;
  double ticks_per_rev_ = 0.0;
  long watchdog_ms_ = 300;
  double telemetry_timeout_s_ = 0.5;
  std::array<std::string, 2> joint_names_;

  // runtime
  SerialPort port_;
  std::string rx_buffer_;
  uint32_t seq_ = 0;
  bool hello_received_ = false;
  bool have_telemetry_ = false;
  rclcpp::Time last_telemetry_time_{0, 0, RCL_STEADY_TIME};
  int64_t last_pico_ms_ = -1;
  std::array<int64_t, 2> last_ticks_{0, 0};
  std::array<int64_t, 2> tick_offset_{0, 0};  // grows when the Pico reboots and its counters restart
  std::array<double, 2> velocity_rad_s_{0.0, 0.0};
  uint64_t rejected_lines_ = 0;
};

}  // namespace karmel_hardware

#endif  // KARMEL_HARDWARE__KARMEL_SYSTEM_HPP_
