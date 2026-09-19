// Lesson 03.09 — enough C++ to read robotics code, on karmel's own telemetry line.
//
// Build and run in a container (no toolchain on your machine, no network needed) — one line:
//   docker run --rm --network none --name m03-cpp -v "$PWD/03-robot-software/code/l0309_cpp:/src:ro"
//       -w /tmp gcc:14 bash -c "g++ -std=c++20 -Wall -Wextra -Wpedantic -O2 /src/telemetry.cpp -o t && ./t"
//
// or natively:  g++ -std=c++20 -Wall -Wextra -Wpedantic -O2 telemetry.cpp -o telemetry && ./telemetry
// or with CMake: cmake -S . -B build && cmake --build build && ./build/karmel_telemetry
//
// This is the same job as robotlab/protocol.py's decode_pico_message, in the language the
// ros2_control plugin (labs/ros2_ws/src/karmel_hardware) is written in. Read them side by side:
// the algorithm is identical, and everything that differs is C++ telling you something about
// memory, ownership or cost.
//
// Deliberately NOT here: exceptions (robot control paths often forbid them), inheritance,
// templates you write yourself, smart pointers. You can read a great deal of robotics C++ with
// only what is below.

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace karmel {

// ---------------------------------------------------------------------------------------------
// Constants are `constexpr`: computed at compile time, no storage, no runtime cost.
// ---------------------------------------------------------------------------------------------
constexpr int kProtocolVersion = 1;
constexpr int kFlagWatchdog = 1;
constexpr int kFlagLowBattery = 2;
constexpr int kFlagRangeError = 4;
constexpr int kFlagVelocityMode = 8;

constexpr double kTicksPerWheelRev = 2464.0;  // 11 CPR x 56 gear x 4 quadrature (karmel.yaml)
constexpr double kWheelRadiusM = 0.045;

// ---------------------------------------------------------------------------------------------
// A plain struct with an explicit integer width for every field. On the wire these ARE the
// widths, and `int` alone does not promise you 32 bits. Python's ints are arbitrary precision;
// here you choose, and choosing wrong overflows silently.
// ---------------------------------------------------------------------------------------------
struct Telemetry {
  std::int64_t ms = 0;             // Pico clock since boot
  std::int64_t left_ticks = 0;     // cumulative, signed: it can go negative
  std::int64_t right_ticks = 0;
  std::int32_t left_mrad_s = 0;
  std::int32_t right_mrad_s = 0;
  std::int32_t battery_mv = -1;    // -1 = no reading (the protocol's "None")
  std::int32_t range_mm = -1;
  std::int32_t flags = 0;
};

// `std::string_view` is a (pointer, length) pair into memory somebody else owns. It is the C++
// answer to "I want to look at this text without copying it" — and the reason it is dangerous is
// that it does not keep the text alive. Never store one longer than the buffer it points into.
std::uint8_t checksum(std::string_view payload) {
  std::uint8_t result = 0;
  for (unsigned char c : payload) {  // `unsigned char`: the XOR is over bytes, not over `char`,
    result ^= c;                     // whose signedness is implementation-defined.
  }
  return result;
}

// `std::optional<T>` is "a T, or nothing" — the same job as Python's `T | None`, but the absence
// is part of the type, so the compiler makes you check before you use it.
std::optional<std::string_view> unframe(std::string_view line) {
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.remove_suffix(1);  // adjusts the view; copies nothing
  }
  const auto star = line.rfind('*');
  if (star == std::string_view::npos || line.size() - star != 3) {
    return std::nullopt;  // no "*hh" tail
  }
  const std::string_view payload = line.substr(0, star);
  const std::string_view hex = line.substr(star + 1);
  unsigned parsed = 0;
  for (char c : hex) {
    const int digit = (c >= '0' && c <= '9')   ? c - '0'
                      : (c >= 'A' && c <= 'F') ? c - 'A' + 10
                      : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                                               : -1;
    if (digit < 0) {
      return std::nullopt;
    }
    parsed = parsed * 16 + static_cast<unsigned>(digit);
  }
  if (checksum(payload) != static_cast<std::uint8_t>(parsed)) {
    return std::nullopt;  // a corrupted line is dropped, never guessed at
  }
  return payload;
}

namespace {  // an anonymous namespace: these names are private to this file (like `_helper`)

// Split on single spaces. Returns views into `payload`, so `payload` must outlive the result —
// this is exactly the kind of lifetime rule C++ asks you to hold in your head.
std::vector<std::string_view> split(std::string_view payload) {
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  while (true) {
    const auto space = payload.find(' ', start);
    if (space == std::string_view::npos) {
      fields.push_back(payload.substr(start));
      return fields;
    }
    fields.push_back(payload.substr(start, space - start));
    start = space + 1;
  }
}

// Strict integer parsing: optional '-', then digits. `out` is an OUT PARAMETER passed by
// reference (`&`) — the classic C++ way to return two things without allocating.
bool parse_int(std::string_view token, std::int64_t& out) {
  const bool negative = !token.empty() && token.front() == '-';
  const std::string_view digits = negative ? token.substr(1) : token;
  if (digits.empty() || digits.size() > 18) {
    return false;
  }
  std::int64_t value = 0;
  for (char c : digits) {
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + (c - '0');
  }
  out = negative ? -value : value;
  return true;
}

}  // namespace

// `const std::string_view&` would be pointless — a view is two words, cheaper to copy than to
// indirect. Pass views and other small values BY VALUE; pass big objects by `const T&`.
std::optional<Telemetry> parse_telemetry(std::string_view payload) {
  const std::vector<std::string_view> f = split(payload);
  if (f.size() != 9 || f[0] != "T") {
    return std::nullopt;
  }
  std::array<std::int64_t, 8> v{};
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (!parse_int(f[i + 1], v[i])) {
      return std::nullopt;
    }
  }
  if (v[0] < 0 || v[5] < -1 || v[6] < -1 || v[7] < 0 || v[7] > 255) {
    return std::nullopt;  // ranges the protocol guarantees; a violated one means a corrupt line
  }
  Telemetry t;
  t.ms = v[0];
  t.left_ticks = v[1];
  t.right_ticks = v[2];
  t.left_mrad_s = static_cast<std::int32_t>(v[3]);   // narrowing is explicit, always
  t.right_mrad_s = static_cast<std::int32_t>(v[4]);
  t.battery_mv = static_cast<std::int32_t>(v[5]);
  t.range_mm = static_cast<std::int32_t>(v[6]);
  t.flags = static_cast<std::int32_t>(v[7]);
  return t;  // returned by value: the compiler moves/elides it, no `new`, no `delete`
}

// ---------------------------------------------------------------------------------------------
// RAII — the one C++ idea that has no C# equivalent worth the name. A destructor runs
// deterministically when the object leaves scope: on `return`, on `break`, and while an exception
// unwinds. That is why a robot's "stop the motors" belongs in one.
//
// C# gets `using` + IDisposable, which you must remember to write at every call site. Here the
// type itself guarantees it. The real KarmelSystem plugin does the same in on_deactivate()/
// on_shutdown(): it sends "S" before the port closes.
// ---------------------------------------------------------------------------------------------
class MotorGuard {
 public:
  explicit MotorGuard(const char* what) : what_(what) {
    std::printf("  [MotorGuard] %s: motors enabled\n", what_);
  }

  ~MotorGuard() { std::printf("  [MotorGuard] %s: STOP sent (destructor)\n", what_); }

  // A guard must not be copied — two copies would each "stop" once. Deleting the copy operations
  // is how C++ says "this type owns something unique". You will see `= delete` everywhere in
  // driver code for exactly this reason.
  MotorGuard(const MotorGuard&) = delete;
  MotorGuard& operator=(const MotorGuard&) = delete;

 private:
  const char* what_;
};

double wheel_speed_rad_s(std::int32_t mrad_s) { return static_cast<double>(mrad_s) / 1000.0; }

double distance_m(std::int64_t left_ticks, std::int64_t right_ticks) {
  const double mean_ticks = static_cast<double>(left_ticks + right_ticks) / 2.0;
  return mean_ticks * 2.0 * M_PI * kWheelRadiusM / kTicksPerWheelRev;
}

}  // namespace karmel

// ---------------------------------------------------------------------------------------------
// A tiny test main(): asserts first, then a readable report. `assert` compiles away with
// -DNDEBUG, which is why release robot code never relies on it for real checks.
// ---------------------------------------------------------------------------------------------
int main() {
  using namespace karmel;

  // The four lines below are real telemetry from labs/exercises/03.07/run_telemetry.csv,
  // re-framed with the protocol's XOR checksum.
  const std::vector<std::string> lines = {
      "T 320 0 0 0 0 12240 -1 1*7D\n",
      "T 1980 4519 4519 8024 8024 12239 2356 8*55\n",
      "T 4200 11158 10508 2626 -8080 12238 1656 8*79\n",
      "T 1980 4519 4519 8024 8024 12239 2356 8*54\n",  // one bit wrong: must be rejected
  };

  // --- unit checks -------------------------------------------------------------------------
  assert(checksum("H 1") == ('H' ^ ' ' ^ '1'));
  assert(!unframe("T 1 2 3").has_value());          // no framing at all
  assert(!unframe(lines[3]).has_value());           // framing present, checksum wrong
  assert(unframe(lines[0]).has_value());
  assert(!parse_telemetry("T 1 2 3").has_value());  // right type, wrong field count
  assert(!parse_telemetry("A 1 OK").has_value());   // not a telemetry message
  {
    const auto payload = unframe(lines[1]);
    assert(payload.has_value());
    const auto t = parse_telemetry(*payload);       // `*opt` unwraps; UB if empty, so check first
    assert(t.has_value());
    assert(t->ms == 1980 && t->left_ticks == 4519 && t->flags == kFlagVelocityMode);
    assert(std::abs(wheel_speed_rad_s(t->left_mrad_s) - 8.024) < 1e-9);
  }
  std::printf("all assertions passed\n\n");

  // --- a report -----------------------------------------------------------------------------
  const MotorGuard guard("demo run");  // its destructor runs at the end of main(), whatever happens

  std::printf("%8s %10s %10s %9s %9s %8s %7s  flags\n", "ms", "left", "right", "l rad/s", "r rad/s",
              "batt V", "dist m");
  int rejected = 0;
  for (const std::string& line : lines) {  // `const&`: no copy of the string, no modification
    const auto payload = unframe(line);
    if (!payload) {
      ++rejected;
      continue;
    }
    const auto t = parse_telemetry(*payload);
    if (!t) {
      ++rejected;
      continue;
    }
    std::printf("%8lld %10lld %10lld %9.3f %9.3f %8.3f %7.3f  %s%s%s%s\n",
                static_cast<long long>(t->ms), static_cast<long long>(t->left_ticks),
                static_cast<long long>(t->right_ticks), wheel_speed_rad_s(t->left_mrad_s),
                wheel_speed_rad_s(t->right_mrad_s), t->battery_mv / 1000.0,
                distance_m(t->left_ticks, t->right_ticks),
                (t->flags & kFlagWatchdog) ? "watchdog " : "",
                (t->flags & kFlagLowBattery) ? "low_battery " : "",
                (t->flags & kFlagRangeError) ? "range_error " : "",
                (t->flags & kFlagVelocityMode) ? "velocity_mode" : "");
  }
  std::printf("\nrejected %d of %zu lines (protocol v%d)\n", rejected, lines.size(),
              kProtocolVersion);
  return 0;
}
