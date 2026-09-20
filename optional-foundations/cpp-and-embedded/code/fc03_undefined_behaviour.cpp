// FC.03 - four bugs a C# developer has never had to think about, and the tools that find them.
//
// Each case is a separate run, because a sanitizer stops the program at the first report.
//   ./ub ok        - the same code written correctly (no report)
//   ./ub dangle    - a reference to an object that is already destroyed
//   ./ub oob       - writing past the end of a vector through operator[]
//   ./ub view      - a std::string_view outliving the string it points into
//   ./ub overflow  - signed integer overflow in an encoder tick counter
//
// Build with the sanitizers on (they cost ~2x speed, so this is a debug build):
//   g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
//       fc03_undefined_behaviour.cpp -o ub
//
// "Undefined behaviour" does not mean "crashes". It means the compiler was allowed to assume
// it could not happen, so anything is a legal outcome - including working on your laptop and
// producing a wrong wheel speed on the robot.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace fc03 {

// --- dangle -----------------------------------------------------------------------------------
struct Wheel {
  double radius_m;
};

// Returns a reference to a local: the Wheel is destroyed when the function returns.
// C# cannot express this bug at all; C++ compiles it with a warning you can ignore.
const Wheel & left_wheel_broken() {
  Wheel w{0.045};
  return w;                       // <- the object dies here
}

const Wheel & left_wheel_ok() {
  static const Wheel w{0.045};    // lives for the whole program
  return w;
}

// --- oob --------------------------------------------------------------------------------------
// operator[] does not check bounds. .at() does, and throws std::out_of_range.
void write_ticks(std::vector<std::int32_t> & buffer, std::size_t index, std::int32_t value, bool checked) {
  if (checked) {
    buffer.at(index) = value;
  } else {
    buffer[index] = value;
  }
}

// --- view -------------------------------------------------------------------------------------
// string_view is a borrowed (pointer, length). It owns nothing and keeps nothing alive.
std::string_view payload_of(const std::string & line) {
  const auto star = line.rfind('*');
  return std::string_view(line).substr(0, star);
}

std::string_view payload_broken() {
  return payload_of(std::string("T 320 0 0 0 0 12240 -1 1*4B"));   // the temporary dies here
}

// --- overflow ---------------------------------------------------------------------------------
// Signed overflow is undefined; unsigned wraps by definition. Encoder counters overflow for real:
// karmel makes 2464 ticks per wheel revolution.
std::int32_t add_ticks_signed(std::int32_t count, std::int32_t delta) { return count + delta; }
std::uint32_t add_ticks_unsigned(std::uint32_t count, std::uint32_t delta) { return count + delta; }

}  // namespace fc03

int main(int argc, char ** argv) {
  using namespace fc03;
  const std::string mode = argc > 1 ? argv[1] : "ok";

  if (mode == "ok") {
    std::printf("wheel radius        : %.3f m\n", left_wheel_ok().radius_m);
    std::vector<std::int32_t> buffer(4, 0);
    write_ticks(buffer, 3, 2464, true);
    std::printf("buffer[3]           : %d\n", buffer[3]);
    const std::string line = "T 320 0 0 0 0 12240 -1 1*4B";
    const std::string_view payload = payload_of(line);      // `line` outlives the view
    std::printf("payload             : %.*s\n", static_cast<int>(payload.size()), payload.data());
    std::printf("unsigned wrap       : %u\n", add_ticks_unsigned(4294967295u, 1u));
    std::puts("no undefined behaviour here");
    return 0;
  }

  if (mode == "dangle") {
    const Wheel & w = left_wheel_broken();
    std::printf("radius read from a destroyed object: %.3f\n", w.radius_m);
    return 0;
  }

  if (mode == "oob") {
    std::vector<std::int32_t> buffer(4, 0);
    write_ticks(buffer, 7, 2464, false);      // three past the end
    std::printf("buffer[0] = %d\n", buffer[0]);
    return 0;
  }

  if (mode == "view") {
    const std::string_view payload = payload_broken();
    std::printf("the view still says it has %zu characters\n", payload.size());
    unsigned checksum = 0;
    for (char c : payload) checksum ^= static_cast<unsigned char>(c);   // reads freed memory
    std::printf("checksum of freed memory: %02X\n", checksum);
    return 0;
  }

  if (mode == "overflow") {
    std::int32_t count = 2147483000;
    for (int i = 0; i < 10; ++i) {
      count = add_ticks_signed(count, 100);   // crosses INT32_MAX = 2147483647
      std::printf("count = %d\n", count);
    }
    return 0;
  }

  std::printf("usage: %s ok|dangle|oob|view|overflow\n", argv[0]);
  return 2;
}
