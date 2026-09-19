#include "karmel_hardware/karmel_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "karmel_hardware/protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace karmel_hardware
{

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace
{
// A steady clock for timeouts: the ROS clock passed to read() may be simulated or jump.
rclcpp::Time steady_now()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock.now();
}
}  // namespace

// =====================================================================================================
// on_init — parse the URDF. Must not touch hardware (it runs even when the robot is not connected,
// e.g. when a tool just loads the robot description).
// =====================================================================================================
KarmelSystem::CallbackReturn KarmelSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // The base class stores the HardwareInfo in info_ and prepares the interfaces declared in the URDF.
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & hw = info_.hardware_parameters;
  auto get = [&hw](const std::string & key, const std::string & fallback) {
      const auto it = hw.find(key);
      return it == hw.end() ? fallback : it->second;
    };
  try {
    device_ = get("device", "");
    baud_ = std::stoi(get("baud", "115200"));
    ticks_per_rev_ = std::stod(get("ticks_per_rev", "0"));
    watchdog_ms_ = std::stol(get("watchdog_ms", "300"));
    telemetry_timeout_s_ = std::stod(get("telemetry_timeout_ms", "500")) / 1000.0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid <param> value in <ros2_control>: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (device_.empty() || ticks_per_rev_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "<param name=\"device\"> and a positive ticks_per_rev are required");
    return CallbackReturn::ERROR;
  }

  // Exactly two joints, each: command velocity; state position + velocity.
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Expected 2 joints (left, right), got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 2; ++i) {
    const auto & joint = info_.joints[i];
    const bool ok = joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == HW_IF_VELOCITY &&
      joint.state_interfaces.size() == 2 &&
      joint.state_interfaces[0].name == HW_IF_POSITION &&
      joint.state_interfaces[1].name == HW_IF_VELOCITY;
    if (!ok) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have command [velocity] and states [position, velocity]",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_[i] = joint.name;
  }
  // By convention the first joint in the URDF is the left wheel.
  RCLCPP_INFO(
    get_logger(), "KarmelSystem: left='%s' right='%s' device=%s ticks/rev=%.0f",
    joint_names_[kLeft].c_str(), joint_names_[kRight].c_str(), device_.c_str(), ticks_per_rev_);
  return CallbackReturn::SUCCESS;
}

// =====================================================================================================
// on_configure — open the port and check we are talking to protocol v1 firmware.
// =====================================================================================================
KarmelSystem::CallbackReturn KarmelSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!port_.open(device_, baud_)) {
    RCLCPP_ERROR(get_logger(), "Cannot open serial port: %s", port_.error().c_str());
    return CallbackReturn::ERROR;
  }
  rx_buffer_.clear();
  hello_received_ = false;
  have_telemetry_ = false;
  last_pico_ms_ = -1;

  // Say hello and wait (up to 2 s) for "I <fw> <protocol>". Blocking is fine here: configure is
  // not part of the real-time loop.
  port_.write(protocol::encode_hello(next_seq()));
  const auto deadline = steady_now() + rclcpp::Duration::from_seconds(2.0);
  while (!hello_received_ && steady_now() < deadline) {
    if (!poll_serial()) {
      RCLCPP_ERROR(get_logger(), "Serial error during hello: %s", port_.error().c_str());
      port_.close();
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!hello_received_) {
    RCLCPP_ERROR(get_logger(), "No hello reply from the Pico on %s (is the firmware running?)", device_.c_str());
    port_.close();
    return CallbackReturn::ERROR;
  }

  port_.write(protocol::encode_param(next_seq(), "watchdog_ms", watchdog_ms_));

  // States start at zero until the first telemetry arrives.
  for (size_t i = 0; i < 2; ++i) {
    set_state(joint_names_[i] + "/" + HW_IF_POSITION, 0.0);
    set_state(joint_names_[i] + "/" + HW_IF_VELOCITY, 0.0);
    set_command(joint_names_[i] + "/" + HW_IF_VELOCITY, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

KarmelSystem::CallbackReturn KarmelSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  port_.close();
  return CallbackReturn::SUCCESS;
}

// =====================================================================================================
// on_activate / on_deactivate — enable and disable motion.
// =====================================================================================================
KarmelSystem::CallbackReturn KarmelSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Never start moving with a command left over from a previous activation.
  for (const auto & name : joint_names_) {
    set_command(name + "/" + HW_IF_VELOCITY, 0.0);
  }
  last_telemetry_time_ = steady_now();  // give the telemetry timeout a fresh start
  RCLCPP_INFO(get_logger(), "KarmelSystem active");
  return CallbackReturn::SUCCESS;
}

KarmelSystem::CallbackReturn KarmelSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (port_.is_open()) {
    port_.write(protocol::encode_stop(next_seq()));
  }
  RCLCPP_INFO(get_logger(), "KarmelSystem inactive, motors stopped");
  return CallbackReturn::SUCCESS;
}

KarmelSystem::CallbackReturn KarmelSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (port_.is_open()) {
    port_.write(protocol::encode_stop(next_seq()));
    port_.close();
  }
  return CallbackReturn::SUCCESS;
}

// =====================================================================================================
// read — telemetry lines → state interfaces.
// =====================================================================================================
KarmelSystem::return_type KarmelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!poll_serial()) {
    RCLCPP_ERROR(get_logger(), "Serial read failed: %s", port_.error().c_str());
    return return_type::ERROR;
  }

  // Stale data is worse than no data: a controller integrating old velocities drives blind.
  if ((steady_now() - last_telemetry_time_).seconds() > telemetry_timeout_s_) {
    RCLCPP_ERROR(get_logger(), "No telemetry from the Pico for %.2f s", telemetry_timeout_s_);
    return return_type::ERROR;
  }

  const double rad_per_tick = 2.0 * M_PI / ticks_per_rev_;
  for (size_t i = 0; i < 2; ++i) {
    set_state(
      joint_names_[i] + "/" + HW_IF_POSITION,
      static_cast<double>(tick_offset_[i] + last_ticks_[i]) * rad_per_tick);
    set_state(joint_names_[i] + "/" + HW_IF_VELOCITY, velocity_rad_s_[i]);
  }
  return return_type::OK;
}

// =====================================================================================================
// write — velocity command interfaces → one "V" line per cycle (which also feeds the Pico watchdog).
// =====================================================================================================
KarmelSystem::return_type KarmelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<int32_t, 2> mrad_s{0, 0};
  for (size_t i = 0; i < 2; ++i) {
    const double command = get_command(joint_names_[i] + "/" + HW_IF_VELOCITY);
    // NaN means "no command yet" in ros2_control; send 0 rather than garbage.
    mrad_s[i] = std::isfinite(command) ? static_cast<int32_t>(std::lround(command * 1000.0)) : 0;
  }
  if (!port_.write(protocol::encode_velocity(next_seq(), mrad_s[kLeft], mrad_s[kRight]))) {
    RCLCPP_ERROR(get_logger(), "Serial write failed: %s", port_.error().c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

// =====================================================================================================
// helpers
// =====================================================================================================
uint32_t KarmelSystem::next_seq()
{
  seq_ = (seq_ + 1) % 65536;
  return seq_;
}

bool KarmelSystem::poll_serial()
{
  if (!port_.read_available(rx_buffer_)) {
    return false;
  }
  size_t newline;
  while ((newline = rx_buffer_.find('\n')) != std::string::npos) {
    handle_line(rx_buffer_.substr(0, newline + 1));
    rx_buffer_.erase(0, newline + 1);
  }
  if (rx_buffer_.size() > 256) {  // garbage without newlines (wrong baud rate?)
    rx_buffer_.clear();
    ++rejected_lines_;
  }
  return true;
}

void KarmelSystem::handle_line(const std::string & line)
{
  const auto payload = protocol::unframe(line);
  if (!payload) {
    ++rejected_lines_;
    return;
  }

  if (auto t = protocol::parse_telemetry(*payload)) {
    if (last_pico_ms_ >= 0 && t->ms < last_pico_ms_) {
      // The Pico's clock went backwards: it rebooted and its encoder counters restarted at 0.
      // Keep the joint position continuous so odometry does not jump.
      tick_offset_[kLeft] += last_ticks_[kLeft];
      tick_offset_[kRight] += last_ticks_[kRight];
      RCLCPP_WARN(get_logger(), "Pico restarted — keeping wheel positions continuous");
    }
    last_pico_ms_ = t->ms;
    last_ticks_ = {t->left_ticks, t->right_ticks};
    velocity_rad_s_ = {t->left_mrad_s / 1000.0, t->right_mrad_s / 1000.0};
    last_telemetry_time_ = steady_now();
    if (!have_telemetry_) {
      have_telemetry_ = true;
      RCLCPP_INFO(get_logger(), "Receiving telemetry (battery %.2f V)", t->battery_mv / 1000.0);
    }
    return;
  }

  if (auto hello = protocol::parse_hello_reply(*payload)) {
    hello_received_ = true;
    if (hello->protocol_version != protocol::kProtocolVersion) {
      RCLCPP_WARN(
        get_logger(), "Pico firmware %s speaks protocol v%d, expected v%d",
        hello->firmware_version.c_str(), hello->protocol_version, protocol::kProtocolVersion);
    } else {
      RCLCPP_INFO(get_logger(), "Pico firmware %s, protocol v1", hello->firmware_version.c_str());
    }
    return;
  }

  if (payload->rfind("A ", 0) == 0 && payload->find(" ERR") != std::string::npos) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Pico rejected a command: %s", payload->c_str());
  }
  // "A <seq> OK" needs no action.
}

}  // namespace karmel_hardware

PLUGINLIB_EXPORT_CLASS(karmel_hardware::KarmelSystem, hardware_interface::SystemInterface)
