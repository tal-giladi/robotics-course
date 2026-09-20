// FC.03 - where C++ objects live, what copying costs, and who owns what.
//
// Build and run (no compiler needed on your machine):
//   docker run --rm --network none --name fc-cpp03
//     -v "$PWD/optional-foundations/cpp-and-embedded/code:/src:ro" -w /tmp gcc:14
//     bash -c "g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 /src/fc03_memory_and_ownership.cpp -o m && ./m"
//   (that is one command; the lesson has the copy-pasteable version)
//
// Nothing here is robotics-specific except the names: Telemetry is karmel's protocol-v1
// telemetry line (labs/README.md) and MotorGuard is the pattern karmel_hardware uses to make
// sure the wheels stop.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fc03 {

// ---------------------------------------------------------------------------- 1. where things live
int g_initialised = 7;   // .data  - lives in the image, copied to RAM at startup
int g_zero;              // .bss   - lives in RAM, zeroed at startup

enum class Region { Static, Stack, Heap, Unknown };

// Classify an address by comparing it with three landmarks we know the region of.
// This is a teaching trick, not something to do in real code: the standard says nothing
// about the relative order of these regions.
Region classify(const void * p, const void * stack_landmark, const void * heap_landmark) {
  const auto a = reinterpret_cast<std::uintptr_t>(p);
  const auto s = reinterpret_cast<std::uintptr_t>(stack_landmark);
  const auto h = reinterpret_cast<std::uintptr_t>(heap_landmark);
  const auto st = reinterpret_cast<std::uintptr_t>(&g_initialised);
  const auto near = [](std::uintptr_t x, std::uintptr_t y) { return (x > y ? x - y : y - x) < (16u << 20); };
  if (near(a, st)) return Region::Static;
  if (near(a, s)) return Region::Stack;
  if (near(a, h)) return Region::Heap;
  return Region::Unknown;
}

const char * name(Region r) {
  switch (r) {
    case Region::Static: return "static (.data/.bss)";
    case Region::Stack:  return "stack";
    case Region::Heap:   return "heap";
    default:             return "somewhere else";
  }
}

// ---------------------------------------------------------------------------- 2. struct layout
// karmel telemetry: T <ms> <lticks> <rticks> <l_mrad_s> <r_mrad_s> <batt_mV> <range_mm> <flags>
struct TelemetryPadded {     // written in the order a human would say it
  std::uint8_t flags;
  std::uint32_t ms;
  std::uint8_t protocol_version;
  std::int32_t left_ticks;
  std::int32_t right_ticks;
  std::int16_t range_mm;
};

struct TelemetryPacked {     // same fields, widest first
  std::uint32_t ms;
  std::int32_t left_ticks;
  std::int32_t right_ticks;
  std::int16_t range_mm;
  std::uint8_t flags;
  std::uint8_t protocol_version;
};

// ---------------------------------------------------------------------------- 3. RAII
// A class whose destructor puts the hardware in a safe state. Nobody has to remember to call
// stop(): leaving the scope - by return, by break, or while an exception unwinds - does it.
class MotorGuard {
 public:
  explicit MotorGuard(const char * who) : who_(who) {
    std::printf("  [MotorGuard %s] motors enabled\n", who_);
  }
  ~MotorGuard() { std::printf("  [MotorGuard %s] STOP sent (destructor)\n", who_); }

  MotorGuard(const MotorGuard &) = delete;             // two guards would stop one robot twice
  MotorGuard & operator=(const MotorGuard &) = delete;

 private:
  const char * who_;
};

void drive_until_it_fails() {
  MotorGuard guard("plan");
  throw std::runtime_error("path blocked");
}

// ---------------------------------------------------------------------------- 4. ownership
class SerialPort {           // stands in for a file descriptor: unique, must be closed exactly once
 public:
  explicit SerialPort(std::string device) : device_(std::move(device)) {
    std::printf("  open(%s)\n", device_.c_str());
  }
  ~SerialPort() { std::printf("  close(%s)\n", device_.c_str()); }
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  const std::string & device() const { return device_; }

 private:
  std::string device_;
};

// ---------------------------------------------------------------------------- 5. a template
// One definition, every arithmetic type. This is all a template is until you write libraries.
template <typename T>
T clamp_to(T value, T low, T high) {
  return value < low ? low : (high < value ? high : value);
}

// ---------------------------------------------------------------------------- helpers
// A volatile sink: writing to it stops the optimiser from deleting work whose result
// is never used. Without it the "copy" benchmark below measures nothing at all.
volatile double g_sink = 0.0;

template <typename F>
double time_us(int repeats, F && body) {
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < repeats; ++i) body();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::micro>(t1 - t0).count() / repeats;
}

}  // namespace fc03

int main() {
  using namespace fc03;

  std::puts("1. Where does each object live?");
  int local = 1;
  auto heap = std::make_unique<int>(2);
  static int function_static = 3;
  const void * s = &local;
  const void * h = heap.get();
  std::printf("  int g_initialised      -> %s\n", name(classify(&g_initialised, s, h)));
  std::printf("  int g_zero             -> %s\n", name(classify(&g_zero, s, h)));
  std::printf("  static int inside main -> %s\n", name(classify(&function_static, s, h)));
  std::printf("  int local              -> %s\n", name(classify(&local, s, h)));
  std::printf("  *make_unique<int>()    -> %s\n", name(classify(heap.get(), s, h)));
  std::vector<double> v(1000, 1.5);
  std::printf("  std::vector object     -> %s, but its 1000 doubles -> %s\n",
              name(classify(&v, s, h)), name(classify(v.data(), s, h)));

  std::puts("\n2. Struct layout: the compiler inserts padding to keep every field aligned.");
  std::printf("  sizeof(TelemetryPadded) = %zu, alignof = %zu\n",
              sizeof(TelemetryPadded), alignof(TelemetryPadded));
  std::printf("  sizeof(TelemetryPacked) = %zu, alignof = %zu  (same 6 fields, reordered)\n",
              sizeof(TelemetryPacked), alignof(TelemetryPacked));
  std::printf("  payload actually used   = %zu bytes; wasted by the bad order = %zu\n",
              sizeof(std::uint32_t) + 2 * sizeof(std::int32_t) + sizeof(std::int16_t) + 2 * sizeof(std::uint8_t),
              sizeof(TelemetryPadded) - sizeof(TelemetryPacked));
  std::printf("  offsets (padded): flags %zu, ms %zu, protocol_version %zu, left_ticks %zu\n",
              offsetof(TelemetryPadded, flags), offsetof(TelemetryPadded, ms),
              offsetof(TelemetryPadded, protocol_version), offsetof(TelemetryPadded, left_ticks));

  std::puts("\n3. Value semantics: a copy copies the bytes, a move steals the pointer.");
  const int reps = 20000;
  const double copy_us = time_us(reps, [&] {
    std::vector<double> c = v;      // 1000 doubles = 8000 bytes really copied
    g_sink = c[999];                // use the result, or the optimiser deletes the copy
  });
  const double move_us = time_us(reps, [&] {
    std::vector<double> src(1000, 1.5);
    std::vector<double> m = std::move(src);   // three pointers change owner; no bytes move
    g_sink = m[999];
  });
  std::printf("  allocate + copy 1000 doubles: %.3f us\n", copy_us);
  std::printf("  allocate + move 1000 doubles: %.3f us   (a move copies no elements)\n", move_us);
  std::printf("  so copying the 8000 bytes costs about %.0f ns; the allocation costs the rest.\n",
              (copy_us - move_us) * 1000.0);
  std::printf("  a C# 'var b = a;' on a List<double> costs neither: it copies one reference.\n");

  std::puts("\n4. RAII: the destructor runs on every way out, including an exception.");
  {
    MotorGuard guard("normal");
    std::puts("  ... driving ...");
  }
  try {
    drive_until_it_fails();
  } catch (const std::exception & e) {
    std::printf("  caught: %s   (the guard already stopped the motors)\n", e.what());
  }

  std::puts("\n5. Ownership: unique_ptr moves, shared_ptr counts.");
  {
    auto port = std::make_unique<SerialPort>("/dev/ttyACM0");
    std::printf("  unique_ptr holds %s\n", port->device().c_str());
    auto moved = std::move(port);
    std::printf("  after std::move: original is %s, new owner holds %s\n",
                port ? "still set" : "empty", moved->device().c_str());
  }  // close() happens here, exactly once
  {
    auto shared = std::make_shared<SerialPort>("/dev/ttyACM1");
    std::printf("  shared_ptr use_count = %ld\n", shared.use_count());
    {
      auto second = shared;
      std::printf("  inside the inner scope use_count = %ld\n", shared.use_count());
    }
    std::printf("  back outside use_count = %ld (nothing closed yet)\n", shared.use_count());
  }  // close() happens here, when the last owner goes away

  std::puts("\n6. One template, several types.");
  std::printf("  clamp_to(17.5, -12.0, 12.0) = %.1f rad/s (karmel's wheel limit)\n",
              clamp_to(17.5, -12.0, 12.0));
  std::printf("  clamp_to(1400, -1000, 1000) = %d per-mille duty\n", clamp_to(1400, -1000, 1000));

  std::puts("\ndone");
  return 0;
}
