#include "karmel_hardware/protocol.hpp"

#include <cctype>
#include <cstdio>
#include <sstream>
#include <vector>

namespace karmel_hardware
{
namespace protocol
{

uint8_t checksum(const std::string & payload)
{
  uint8_t result = 0;
  for (unsigned char c : payload) {
    result ^= c;
  }
  return result;
}

std::string frame(const std::string & payload)
{
  char hex[4];
  std::snprintf(hex, sizeof(hex), "%02X", checksum(payload));
  return payload + "*" + hex + "\n";
}

std::optional<std::string> unframe(const std::string & line)
{
  std::string text = line;
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
    text.pop_back();
  }
  const auto star = text.rfind('*');
  if (star == std::string::npos || text.size() - star != 3) {
    return std::nullopt;
  }
  const std::string hex = text.substr(star + 1);
  for (char c : hex) {
    if (!std::isxdigit(static_cast<unsigned char>(c))) {
      return std::nullopt;  // emitted uppercase, accepted in either case (robotlab/protocol.py)
    }
  }
  const std::string payload = text.substr(0, star);
  if (checksum(payload) != static_cast<uint8_t>(std::stoul(hex, nullptr, 16))) {
    return std::nullopt;
  }
  return payload;
}

std::string encode_hello(uint32_t seq)
{
  return frame("H " + std::to_string(seq));
}

std::string encode_velocity(uint32_t seq, int32_t left_mrad_s, int32_t right_mrad_s)
{
  return frame(
    "V " + std::to_string(seq) + " " + std::to_string(left_mrad_s) + " " +
    std::to_string(right_mrad_s));
}

std::string encode_stop(uint32_t seq)
{
  return frame("S " + std::to_string(seq));
}

std::string encode_param(uint32_t seq, const std::string & key, long value)
{
  return frame("P " + std::to_string(seq) + " " + key + " " + std::to_string(value));
}

namespace
{
// Split on single spaces, exactly as the Python implementation does.
std::vector<std::string> split(const std::string & payload)
{
  std::vector<std::string> fields;
  std::string current;
  for (char c : payload) {
    if (c == ' ') {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  fields.push_back(current);
  return fields;
}

// Strict integer: optional '-', then digits only.
bool parse_int(const std::string & token, int64_t & out)
{
  size_t start = (!token.empty() && token[0] == '-') ? 1 : 0;
  if (token.size() == start || token.size() > 19) {
    return false;
  }
  for (size_t i = start; i < token.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(token[i]))) {
      return false;
    }
  }
  out = std::stoll(token);
  return true;
}
}  // namespace

std::optional<Telemetry> parse_telemetry(const std::string & payload)
{
  const auto f = split(payload);
  if (f.size() != 9 || f[0] != "T") {
    return std::nullopt;
  }
  int64_t v[8];
  for (int i = 0; i < 8; ++i) {
    if (!parse_int(f[i + 1], v[i])) {
      return std::nullopt;
    }
  }
  if (v[0] < 0 || v[5] < -1 || v[6] < -1 || v[7] < 0 || v[7] > 255) {
    return std::nullopt;
  }
  Telemetry t;
  t.ms = v[0];
  t.left_ticks = v[1];
  t.right_ticks = v[2];
  t.left_mrad_s = static_cast<int32_t>(v[3]);
  t.right_mrad_s = static_cast<int32_t>(v[4]);
  t.battery_mv = static_cast<int32_t>(v[5]);
  t.range_mm = static_cast<int32_t>(v[6]);
  t.flags = static_cast<int32_t>(v[7]);
  return t;
}

std::optional<HelloReply> parse_hello_reply(const std::string & payload)
{
  const auto f = split(payload);
  int64_t version = 0;
  if (f.size() != 3 || f[0] != "I" || f[1].empty() || !parse_int(f[2], version)) {
    return std::nullopt;
  }
  return HelloReply{f[1], static_cast<int>(version)};
}

}  // namespace protocol
}  // namespace karmel_hardware
