// FC.08 - karmel's wheel node, as a micro-ROS node running on the Pico itself.
//
// This is the firmware half of the lesson. It replaces the ASCII protocol of
// labs/firmware/pico/ with a ROS 2 client on the microcontroller:
//
//   subscribes  /karmel/cmd_vel     geometry_msgs/msg/Twist      (body velocity)
//   publishes   /karmel/joint_states sensor_msgs/msg/JointState  at 50 Hz
//
// Build it by overlaying this file on micro-ROS's own Pico SDK example repository
// (see CMakeLists.txt next to this file and the lesson's Code section).
//
// SAFETY: this firmware never touches the motor driver pins. `motors_set_duty()`
// drives the on-board LED through PWM so that the whole program can be flashed onto a
// Pico that is bolted to the robot without the wheels moving. The encoder feedback is
// simulated by integrating the commanded speed - on the real robot those two numbers
// come from the PIO counters of FC.05.
//
// Three things in here are the lesson:
//   1. the agent-connection state machine (WAITING_AGENT .. AGENT_DISCONNECTED);
//   2. the 100 Hz control step and the command watchdog run in the SUPERLOOP, not in an
//      rclc timer, so they keep running when the middleware is gone;
//   3. every allocation happens in create_entities(), never in a callback.

#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "pico_uart_transports.h"

// --- karmel's numbers (labs/config/karmel.yaml, labs/firmware/pico/config.py) --------------
#define WHEEL_RADIUS_M 0.045f
#define WHEEL_SEPARATION_M 0.200f
#define MAX_WHEEL_SPEED_RAD_S 17.0f
#define WATCHDOG_MS 300
#define CONTROL_PERIOD_MS 10   // 100 Hz, config.CONTROL_HZ
#define TELEMETRY_PERIOD_MS 20 // 50 Hz, config.TELEMETRY_HZ
#define LED_PIN 25
#define PWM_WRAP 999

#define RCCHECK(fn)                      \
  {                                      \
    rcl_ret_t rc_ = fn;                  \
    if (rc_ != RCL_RET_OK) return false; \
  }
#define RCSOFTCHECK(fn)   \
  {                       \
    rcl_ret_t rc_ = fn;   \
    (void)rc_;            \
  }

#define EXECUTE_EVERY_N_MS(MS, X)                            \
  {                                                          \
    static int64_t last_ms_ = -1;                            \
    int64_t now_ms_ = (int64_t)(time_us_64() / 1000);        \
    if (last_ms_ == -1) last_ms_ = now_ms_;                  \
    if ((now_ms_ - last_ms_) > (MS)) { X; last_ms_ = now_ms_; } \
  }

typedef enum {
  WAITING_AGENT,     // no agent on the wire: ping every 500 ms, motors stopped
  AGENT_AVAILABLE,   // a ping came back: build node, publisher, subscription, timer
  AGENT_CONNECTED,   // spinning; ping every 200 ms to notice a silent death
  AGENT_DISCONNECTED // brake first, then tear the entities down
} agent_state_t;

// --- ROS entities: created in create_entities(), destroyed in destroy_entities() -----------
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t joint_pub;
static rcl_subscription_t cmd_sub;
static rcl_timer_t telemetry_timer;

static geometry_msgs__msg__Twist cmd_msg;      // filled by the subscription callback
static sensor_msgs__msg__JointState joint_msg; // published at 50 Hz

static double joint_position[2];               // wheel angle, rad
static double joint_velocity[2];               // wheel speed, rad/s
static rosidl_runtime_c__String joint_names[2];

// --- state shared between the superloop and the callbacks ---------------------------------
static volatile float setpoint_rad_s[2] = {0.0f, 0.0f};
static float duty[2] = {0.0f, 0.0f};
static uint32_t last_cmd_ms = 0;
static bool watchdog_tripped = true;           // starts tripped, exactly like FC.07's rule

// --- "motors": the on-board LED, never the driver pins ------------------------------------
static void motors_init(void) {
  gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(LED_PIN);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, PWM_WRAP);
  pwm_init(slice, &config, true);
}

static void motors_set_duty(float left, float right) {
  duty[0] = left;
  duty[1] = right;
  float magnitude = (left < 0.0f ? -left : left);
  if (magnitude > 1.0f) magnitude = 1.0f;
  pwm_set_gpio_level(LED_PIN, (uint16_t)(magnitude * PWM_WRAP));
}

static void motors_brake(void) {
  setpoint_rad_s[0] = 0.0f;
  setpoint_rad_s[1] = 0.0f;
  motors_set_duty(0.0f, 0.0f);
}

// --- callbacks: short, no allocation, no printf -------------------------------------------
static void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  const float v = (float)msg->linear.x;        // m/s
  const float w = (float)msg->angular.z;       // rad/s
  // Differential drive inverse kinematics (lesson 02.03 / 08.10).
  float left = (v - w * WHEEL_SEPARATION_M * 0.5f) / WHEEL_RADIUS_M;
  float right = (v + w * WHEEL_SEPARATION_M * 0.5f) / WHEEL_RADIUS_M;
  if (left > MAX_WHEEL_SPEED_RAD_S) left = MAX_WHEEL_SPEED_RAD_S;
  if (left < -MAX_WHEEL_SPEED_RAD_S) left = -MAX_WHEEL_SPEED_RAD_S;
  if (right > MAX_WHEEL_SPEED_RAD_S) right = MAX_WHEEL_SPEED_RAD_S;
  if (right < -MAX_WHEEL_SPEED_RAD_S) right = -MAX_WHEEL_SPEED_RAD_S;
  setpoint_rad_s[0] = left;
  setpoint_rad_s[1] = right;
  last_cmd_ms = (uint32_t)(time_us_64() / 1000);
  watchdog_tripped = false;
}

static void telemetry_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;
  int64_t stamp_ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos()
                                                   : (int64_t)time_us_64() * 1000;
  joint_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
  joint_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);
  joint_msg.position.data[0] = joint_position[0];
  joint_msg.position.data[1] = joint_position[1];
  joint_msg.velocity.data[0] = joint_velocity[0];
  joint_msg.velocity.data[1] = joint_velocity[1];
  RCSOFTCHECK(rcl_publish(&joint_pub, &joint_msg, NULL));
}

// --- the control step: 100 Hz, independent of the middleware -------------------------------
static void control_step(uint32_t now_ms) {
  if (!watchdog_tripped && (uint32_t)(now_ms - last_cmd_ms) >= WATCHDOG_MS) {
    watchdog_tripped = true;     // FC.07: report once, then stay tripped until the next command
    motors_brake();
  }
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  for (int i = 0; i < 2; ++i) {
    // Stand-in for the PID of labs/firmware/pico/velocity.py: on the real robot the measured
    // speed comes from the PIO encoder counters. Here the wheel follows its setpoint exactly.
    joint_velocity[i] = watchdog_tripped ? 0.0 : (double)setpoint_rad_s[i];
    joint_position[i] += joint_velocity[i] * dt;
  }
  motors_set_duty(watchdog_tripped ? 0.0f : setpoint_rad_s[0] / MAX_WHEEL_SPEED_RAD_S,
                  watchdog_tripped ? 0.0f : setpoint_rad_s[1] / MAX_WHEEL_SPEED_RAD_S);
}

// --- entity lifecycle ----------------------------------------------------------------------
static bool create_entities(void) {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "karmel_wheels", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &joint_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "karmel/joint_states"));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "karmel/cmd_vel"));

  RCCHECK(rclc_timer_init_default2(&telemetry_timer, &support,
                                   RCL_MS_TO_NS(TELEMETRY_PERIOD_MS),
                                   telemetry_callback, true));

  // Two handles: one subscription + one timer. Publishers are not handles.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &telemetry_timer));

  // All message memory is allocated ONCE, here. Nothing allocates in a callback.
  rosidl_runtime_c__String__assign(&joint_msg.header.frame_id, "base_link");
  rosidl_runtime_c__String__assign(&joint_names[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[1], "right_wheel_joint");
  joint_msg.name.data = joint_names;
  joint_msg.name.size = 2;
  joint_msg.name.capacity = 2;
  joint_msg.position.data = joint_position;
  joint_msg.position.size = 2;
  joint_msg.position.capacity = 2;
  joint_msg.velocity.data = joint_velocity;
  joint_msg.velocity.size = 2;
  joint_msg.velocity.capacity = 2;

  // Ask the agent for the ROS 2 epoch so the header stamps mean something on the Pi.
  RCSOFTCHECK(rmw_uros_sync_session(1000));
  return true;
}

static void destroy_entities(void) {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&joint_pub, &node));
  RCSOFTCHECK(rcl_subscription_fini(&cmd_sub, &node));
  RCSOFTCHECK(rcl_timer_fini(&telemetry_timer));
  rclc_executor_fini(&executor);
  RCSOFTCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);
}

int main(void) {
  rmw_uros_set_custom_transport(true, NULL,
                                pico_serial_transport_open,
                                pico_serial_transport_close,
                                pico_serial_transport_write,
                                pico_serial_transport_read);
  motors_init();
  motors_brake();

  agent_state_t state = WAITING_AGENT;
  uint32_t next_control_ms = (uint32_t)(time_us_64() / 1000);

  while (true) {
    const uint32_t now_ms = (uint32_t)(time_us_64() / 1000);

    // 1. Safety first: the control step and the watchdog never depend on the middleware.
    if ((int32_t)(now_ms - next_control_ms) >= 0) {
      control_step(now_ms);
      next_control_ms += CONTROL_PERIOD_MS;     // keep the grid, do not drift (FC.07)
      if ((int32_t)(now_ms - next_control_ms) >= 0) next_control_ms = now_ms + CONTROL_PERIOD_MS;
    }

    // 2. Then the ROS 2 session.
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                            ? AGENT_AVAILABLE
                                            : WAITING_AGENT;);
        break;
      case AGENT_AVAILABLE:
        state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) destroy_entities();
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                            ? AGENT_CONNECTED
                                            : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        }
        break;
      case AGENT_DISCONNECTED:
        motors_brake();          // the robot stops BEFORE we spend time tearing entities down
        watchdog_tripped = true;
        destroy_entities();
        state = WAITING_AGENT;
        break;
    }
  }
  return 0;
}
