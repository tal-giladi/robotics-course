#include <gtest/gtest.h>

#include <cctype>
#include <string>

#include "karmel_hardware/protocol.hpp"

using karmel_hardware::protocol::checksum;
using karmel_hardware::protocol::encode_hello;
using karmel_hardware::protocol::encode_param;
using karmel_hardware::protocol::encode_stop;
using karmel_hardware::protocol::encode_velocity;
using karmel_hardware::protocol::frame;
using karmel_hardware::protocol::parse_hello_reply;
using karmel_hardware::protocol::parse_telemetry;
using karmel_hardware::protocol::unframe;

TEST(Protocol, ChecksumIsXor)
{
  EXPECT_EQ(checksum(""), 0);
  EXPECT_EQ(checksum("A"), 0x41);
  EXPECT_EQ(checksum("AB"), 0x41 ^ 0x42);
}

TEST(Protocol, FrameAndUnframe)
{
  const std::string line = frame("S 5");
  EXPECT_EQ(line.back(), '\n');
  EXPECT_EQ(line.substr(0, 4), "S 5*");
  ASSERT_TRUE(unframe(line).has_value());
  EXPECT_EQ(*unframe(line), "S 5");
  EXPECT_EQ(*unframe(line.substr(0, line.size() - 1) + "\r\n"), "S 5");
}

TEST(Protocol, AcceptsLowercaseChecksumDigits)
{
  // robotlab/protocol.py: checksums are emitted uppercase, accepted in either case.
  for (const std::string payload : {"V 1 10 20", "T 1 2 3 4 5 12000 100 0", "A 7 OK", "I fw 1"}) {
    std::string line = frame(payload);
    for (size_t i = line.size() - 3; i < line.size() - 1; ++i) {
      line[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(line[i])));
    }
    ASSERT_TRUE(unframe(line).has_value()) << line;
    EXPECT_EQ(*unframe(line), payload);
  }
}

TEST(Protocol, RejectsBadLines)
{
  EXPECT_FALSE(unframe("S 5\n").has_value());
  EXPECT_FALSE(unframe("S 5*0\n").has_value());
  std::string line = frame("T 1 2 3 4 5 12000 100 0");
  line[2] ^= 0x01;
  EXPECT_FALSE(unframe(line).has_value());
}

// Byte-identical to karmel_base/protocol.py (Python): encode(Velocity(7, 1500, -1500)) etc.
TEST(Protocol, EncodesLikeThePythonImplementation)
{
  EXPECT_EQ(encode_hello(1), frame("H 1"));
  EXPECT_EQ(encode_velocity(7, 1500, -1500), frame("V 7 1500 -1500"));
  EXPECT_EQ(encode_stop(9), frame("S 9"));
  EXPECT_EQ(encode_param(3, "watchdog_ms", 300), frame("P 3 watchdog_ms 300"));
  // checksum literal computed independently: XOR("H 1") = 0x48 ^ 0x20 ^ 0x31 = 0x59
  EXPECT_EQ(encode_hello(1), "H 1*59\n");
}

TEST(Protocol, ParsesTelemetry)
{
  const auto t = parse_telemetry("T 123456 -2464 1232 6283 -3141 11850 -1 13");
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->ms, 123456);
  EXPECT_EQ(t->left_ticks, -2464);
  EXPECT_EQ(t->right_ticks, 1232);
  EXPECT_EQ(t->left_mrad_s, 6283);
  EXPECT_EQ(t->right_mrad_s, -3141);
  EXPECT_EQ(t->battery_mv, 11850);
  EXPECT_EQ(t->range_mm, -1);
  EXPECT_EQ(t->flags, 13);
}

TEST(Protocol, RejectsMalformedTelemetry)
{
  EXPECT_FALSE(parse_telemetry("T 1 2 3 4 5 6 7").has_value());
  EXPECT_FALSE(parse_telemetry("T 1 2 3 4 5 6 7 8 9").has_value());
  EXPECT_FALSE(parse_telemetry("T 1 2 3 4.5 5 6 7 8").has_value());
  EXPECT_FALSE(parse_telemetry("T 1 2 3 4 5 6 -2 8").has_value());
  EXPECT_FALSE(parse_telemetry("T 1 2 3 4 5 6 7 256").has_value());
  EXPECT_TRUE(parse_telemetry("T 1 2 3 4 5 -1 -1 0").has_value());  // no battery reading
  EXPECT_FALSE(parse_telemetry("A 1 OK").has_value());
}

TEST(Protocol, ParsesHelloReply)
{
  const auto h = parse_hello_reply("I 1.2.0 1");
  ASSERT_TRUE(h.has_value());
  EXPECT_EQ(h->firmware_version, "1.2.0");
  EXPECT_EQ(h->protocol_version, 1);
  EXPECT_FALSE(parse_hello_reply("I 1.2.0").has_value());
}
