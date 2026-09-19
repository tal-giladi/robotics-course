// Pi <-> Pico serial protocol v1 — the few pieces the ros2_control plugin needs.
//
// Wire format (labs/README.md):   <payload>*<hh>\n    hh = XOR of all payload bytes, uppercase hex
//
// The canonical implementation (every message, both directions) is Python:
// labs/python/robotlab/protocol.py (copied into karmel_base); this mirrors the parts ros2_control needs.
#ifndef KARMEL_HARDWARE__PROTOCOL_HPP_
#define KARMEL_HARDWARE__PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace karmel_hardware
{
namespace protocol
{

constexpr int kProtocolVersion = 1;

// Telemetry flag bits
constexpr int kFlagWatchdog = 1;
constexpr int kFlagLowBattery = 2;
constexpr int kFlagRangeError = 4;
constexpr int kFlagVelocityMode = 8;

/// XOR of all payload bytes.
uint8_t checksum(const std::string & payload);

/// "<payload>*<hh>\n"
std::string frame(const std::string & payload);

/// Validate "<payload>*<hh>[\r]\n" and return the payload, or nullopt if framing/checksum is wrong.
std::optional<std::string> unframe(const std::string & line);

// ---- Host -> Pico --------------------------------------------------------------------------
std::string encode_hello(uint32_t seq);                                         // H <seq>
std::string encode_velocity(uint32_t seq, int32_t left_mrad_s, int32_t right_mrad_s);  // V ...
std::string encode_stop(uint32_t seq);                                          // S <seq>
std::string encode_param(uint32_t seq, const std::string & key, long value);    // P <seq> <key> <value>

// ---- Pico -> host --------------------------------------------------------------------------
struct Telemetry
{
  int64_t ms = 0;           // Pico clock
  int64_t left_ticks = 0;   // cumulative encoder ticks
  int64_t right_ticks = 0;
  int32_t left_mrad_s = 0;  // wheel velocity estimate
  int32_t right_mrad_s = 0;
  int32_t battery_mv = -1;  // -1 = no reading
  int32_t range_mm = -1;    // -1 = invalid
  int32_t flags = 0;
};

struct HelloReply
{
  std::string firmware_version;
  int protocol_version = 0;
};

/// Parse a "T ..." payload (already unframed). nullopt if it is not a well-formed telemetry message.
std::optional<Telemetry> parse_telemetry(const std::string & payload);

/// Parse an "I <firmware_version> <protocol_version>" payload.
std::optional<HelloReply> parse_hello_reply(const std::string & payload);

}  // namespace protocol
}  // namespace karmel_hardware

#endif  // KARMEL_HARDWARE__PROTOCOL_HPP_
