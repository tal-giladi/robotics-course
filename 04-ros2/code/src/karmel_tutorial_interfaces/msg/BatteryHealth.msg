# Health summary of karmel's 3S Li-ion pack, derived from the raw voltage.
# Published by karmel_tutorial/battery_health on /battery/health (lesson 04.06).
#
# The raw measurement belongs in sensor_msgs/BatteryState (lesson 04.16). This message adds
# what BatteryState does not have: a filtered voltage, a coarse health state with hysteresis,
# and the threshold that produced it.

# Health states (constants: generated as class attributes, e.g. BatteryHealth.STATE_LOW)
uint8 STATE_OK=0
uint8 STATE_LOW=1
uint8 STATE_CRITICAL=2

std_msgs/Header header      # stamp = when the voltage sample was processed
float32 voltage_v           # low-pass filtered pack voltage [V]
float32 raw_voltage_v       # last unfiltered sample [V]
float32 percent             # 0..100, crude linear estimate between cutoff and full
uint8 state                 # one of STATE_*
float32 low_threshold_v     # threshold in force when this message was produced [V]
