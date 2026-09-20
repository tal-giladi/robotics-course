// FC.06 - the same three jobs karmel's MicroPython firmware does, written against the Pico SDK.
//
//   * blink the status LED from a repeating hardware timer (no CPU spinning)
//   * generate 20 kHz PWM and ramp its duty, using the same PWM slice karmel uses for a motor
//   * measure how long integer, fixed-point, float and double arithmetic take on THIS chip
//
// It never touches the motor pins: the PWM output goes to the on-board LED (GPIO 25), so you can
// run it on a Pico that is plugged into the robot without the wheels moving.
//
// Build: see CMakeLists.txt in this folder. Flash: drag build/fc06_firmware.uf2 onto the
// RP2350 drive, then `mpremote`-free serial: `minicom -D /dev/ttyACM0 -b 115200` or
// `python -m serial.tools.miniterm /dev/ttyACM0 115200`.

#include <cinttypes>
#include <cstdint>
#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

namespace {

constexpr uint kLedPin = PICO_DEFAULT_LED_PIN;   // GPIO 25 on a Pico 2 (config.STATUS_LED)
constexpr uint32_t kPwmHz = 20'000;              // config.PWM_FREQ_HZ
constexpr int kBenchIterations = 200'000;

volatile uint32_t g_blinks = 0;                  // written by the timer callback, read by main

// --- 1. a repeating hardware timer -------------------------------------------------------------
// This callback runs in interrupt context. The rules from FC.05 apply exactly: keep it short,
// do not printf, and anything it shares with main() is `volatile` (and, for anything wider than
// a word, needs a critical section).
bool on_blink_timer(repeating_timer_t *) {
  gpio_xor_mask(1u << kLedPin);                  // the single register write from FC.01
  ++g_blinks;
  return true;                                   // false would cancel the timer
}

// --- 2. PWM ------------------------------------------------------------------------------------
// The RP2350 PWM counter runs at clk_sys (150 MHz) divided by `divider`, and wraps at `wrap`.
//     f_pwm = clk_sys / (divider * (wrap + 1))
// We want 20 kHz with the finest possible duty resolution, so pick divider = 1 and
// wrap = clk_sys / f_pwm - 1.
uint pwm_init_20khz(uint gpio) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  const uint slice = pwm_gpio_to_slice_num(gpio);
  const uint32_t wrap = clock_get_hz(clk_sys) / kPwmHz - 1;
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&config, 1);
  pwm_config_set_wrap(&config, static_cast<uint16_t>(wrap));
  pwm_init(slice, &config, true);
  pwm_set_gpio_level(gpio, 0);
  printf("PWM: clk_sys %" PRIu32 " Hz, wrap %" PRIu32 " -> %.1f Hz, %" PRIu32 " duty steps\n",
         clock_get_hz(clk_sys), wrap,
         static_cast<double>(clock_get_hz(clk_sys)) / (wrap + 1), wrap + 1);
  return slice;
}

// --- 3. arithmetic benchmark -------------------------------------------------------------------
// Q16.16 fixed point: one int32 holds value * 65536. Multiplication needs 64 bits in the middle,
// which is why fixed point is only cheap when the compiler has a 32x32->64 multiply (it does).
using q16_16 = std::int32_t;
constexpr q16_16 to_q16(double v) { return static_cast<q16_16>(v * 65536.0 + (v >= 0 ? 0.5 : -0.5)); }
constexpr double from_q16(q16_16 v) { return static_cast<double>(v) / 65536.0; }
inline q16_16 q16_mul(q16_16 a, q16_16 b) {
  return static_cast<q16_16>((static_cast<std::int64_t>(a) * b) >> 16);
}

// `volatile` sinks so the optimiser cannot delete the loops.
volatile std::int32_t g_sink_i = 0;
volatile q16_16 g_sink_q = 0;
volatile float g_sink_f = 0.0f;
volatile double g_sink_d = 0.0;

template <typename Body>
uint32_t bench_us(Body && body) {
  const absolute_time_t t0 = get_absolute_time();
  body();
  return static_cast<uint32_t>(absolute_time_diff_us(t0, get_absolute_time()));
}

void run_benchmark() {
  // One "PID step" worth of arithmetic: error -> P term -> accumulate -> clamp.
  const std::int32_t kp_i = 328;                 // 0.02 * 16384, a crude fixed scale
  const q16_16 kp_q = to_q16(0.02);
  const float kp_f = 0.02f;
  const double kp_d = 0.02;

  const uint32_t us_int = bench_us([&] {
    std::int32_t acc = 0;
    for (int i = 0; i < kBenchIterations; ++i) acc = (acc + kp_i * (i & 0xFF)) >> 1;
    g_sink_i = acc;
  });
  const uint32_t us_q16 = bench_us([&] {
    q16_16 acc = 0;
    for (int i = 0; i < kBenchIterations; ++i) acc = (acc + q16_mul(kp_q, to_q16(i & 0xFF))) >> 1;
    g_sink_q = acc;
  });
  const uint32_t us_flt = bench_us([&] {
    float acc = 0.0f;
    for (int i = 0; i < kBenchIterations; ++i) acc = (acc + kp_f * static_cast<float>(i & 0xFF)) * 0.5f;
    g_sink_f = acc;
  });
  const uint32_t us_dbl = bench_us([&] {
    double acc = 0.0;
    for (int i = 0; i < kBenchIterations; ++i) acc = (acc + kp_d * static_cast<double>(i & 0xFF)) * 0.5;
    g_sink_d = acc;
  });

  const double per = static_cast<double>(kBenchIterations);
  printf("\n%-14s %10s %12s\n", "arithmetic", "total us", "ns per op");
  printf("%-14s %10" PRIu32 " %12.1f\n", "int32", us_int, us_int * 1000.0 / per);
  printf("%-14s %10" PRIu32 " %12.1f\n", "Q16.16 fixed", us_q16, us_q16 * 1000.0 / per);
  printf("%-14s %10" PRIu32 " %12.1f\n", "float", us_flt, us_flt * 1000.0 / per);
  printf("%-14s %10" PRIu32 " %12.1f\n", "double", us_dbl, us_dbl * 1000.0 / per);
  printf("Q16.16 sanity: 0.02 stored as %" PRId32 " = %.6f\n", kp_q, from_q16(kp_q));
}

}  // namespace

int main() {
  stdio_init_all();
  sleep_ms(2000);                       // give the host time to open the USB serial port

  printf("\nFC.06 firmware, Pico SDK build\n");
  printf("reset caused by the watchdog: %s\n", watchdog_caused_reboot() ? "yes" : "no");

  gpio_init(kLedPin);
  gpio_set_dir(kLedPin, GPIO_OUT);

  repeating_timer_t blink_timer;
  add_repeating_timer_ms(-250, on_blink_timer, nullptr, &blink_timer);   // negative = period, not delay

  run_benchmark();

  const uint32_t blinks_after_benchmark = g_blinks;
  printf("\nthe timer fired %" PRIu32 " times while the benchmark ran - the CPU never waited for it\n",
         blinks_after_benchmark);

  cancel_repeating_timer(&blink_timer);
  const uint slice = pwm_init_20khz(kLedPin);
  const uint16_t top = static_cast<uint16_t>(clock_get_hz(clk_sys) / kPwmHz - 1);
  printf("ramping the LED with 20 kHz PWM on slice %u\n", slice);
  for (int pass = 0; pass < 2; ++pass) {
    for (uint16_t level = 0; level <= top; level = static_cast<uint16_t>(level + top / 50 + 1)) {
      pwm_set_gpio_level(kLedPin, level);
      sleep_ms(20);
    }
  }
  pwm_set_gpio_level(kLedPin, 0);

  printf("done - idling. Press the RESET/BOOTSEL combination to reflash.\n");
  while (true) {
    tight_loop_contents();
  }
}
