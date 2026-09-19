"""07.03 / 07.04 — The physics numbers behind ultrasonic and optical time-of-flight ranging.

    py 07-sensors/code/range_physics.py            # prints every table used in the lessons

Everything here is a small pure function so the lessons' numbers are computed, not typed.
"""

from __future__ import annotations

import math

SPEED_OF_LIGHT_M_S = 299_792_458.0
FIRMWARE_SPEED_OF_SOUND_M_S = 343.0  # labs/firmware/pico/config.py SPEED_OF_SOUND_M_S (dry air, ~20 °C)


# --- sound ------------------------------------------------------------------------------------------
def speed_of_sound(air_temp_c: float) -> float:
    """Speed of sound in dry air (m/s): c = 331.3 * sqrt(1 + T/273.15). Humidity adds < 0.5 %."""
    return 331.3 * math.sqrt(1.0 + air_temp_c / 273.15)


def echo_time_s(distance_m: float, air_temp_c: float) -> float:
    """Round-trip time of a ping to a target ``distance_m`` away and back."""
    return 2.0 * distance_m / speed_of_sound(air_temp_c)


def ultrasonic_reading_m(true_distance_m: float, air_temp_c: float,
                         assumed_speed_m_s: float = FIRMWARE_SPEED_OF_SOUND_M_S) -> float:
    """What a sensor that assumes ``assumed_speed_m_s`` reports for a target at the true distance."""
    return echo_time_s(true_distance_m, air_temp_c) * assumed_speed_m_s / 2.0


def beam_footprint_radius_m(distance_m: float, full_angle_deg: float) -> float:
    """Radius of the circle a cone of ``full_angle_deg`` covers at ``distance_m``."""
    return distance_m * math.tan(math.radians(full_angle_deg) / 2.0)


def max_ping_rate_hz(max_range_m: float, air_temp_c: float = 20.0, decay_factor: float = 1.5) -> float:
    """Highest safe ping rate: wait for the farthest echo, times a margin for reverberation to die out."""
    return 1.0 / (echo_time_s(max_range_m, air_temp_c) * decay_factor)


def blind_zone_m(ring_down_s: float, air_temp_c: float = 20.0) -> float:
    """Closest measurable distance when the transducer keeps ringing for ``ring_down_s`` after the ping."""
    return speed_of_sound(air_temp_c) * ring_down_s / 2.0


def specular_miss_distance_m(distance_m: float, incidence_deg: float) -> float:
    """How far to the side a mirror-like echo returns: the reflected ray leaves at 2 x incidence.

    Where the central ray's echo crosses the sensor plane again, measured from the sensor. If this
    is much larger than the transducer (a few cm), that ray's echo misses the receiver.
    """
    if abs(incidence_deg) >= 45.0:
        return math.inf  # the echo leaves parallel to the wall or away from the sensor
    return distance_m * math.tan(math.radians(2.0 * abs(incidence_deg)))


# --- light ------------------------------------------------------------------------------------------
def light_round_trip_s(distance_m: float) -> float:
    return 2.0 * distance_m / SPEED_OF_LIGHT_M_S


def timing_resolution_for(distance_resolution_m: float) -> float:
    """Clock resolution needed to resolve a distance step with light (s)."""
    return light_round_trip_s(distance_resolution_m)


def relative_return_signal(distance_m: float, reference_m: float = 1.0) -> float:
    """Returned photons from a large diffuse target scale as 1/d^2 (relative to ``reference_m``)."""
    return (reference_m / distance_m) ** 2


def shot_noise_sigma_scale(distance_m: float, reference_m: float = 1.0, ambient_fraction: float = 0.0) -> float:
    """Relative ranging sigma when noise is photon (shot) noise: sigma ∝ sqrt(signal + ambient) / signal.

    ``ambient_fraction`` is ambient photons as a fraction of the signal at the reference distance.
    """
    signal = relative_return_signal(distance_m, reference_m)
    at_reference = math.sqrt(1.0 + ambient_fraction)
    return math.sqrt(signal + ambient_fraction) / signal / at_reference


def vl53l1x_roi_fov_deg(spads: int, full_spads: int = 16, full_fov_deg: float = 27.0) -> float:
    """Approximate field of view for a square ROI of ``spads`` x ``spads`` SPADs (linear scaling)."""
    return full_fov_deg * spads / full_spads


def tables() -> str:
    out = ["Speed of sound and the US-100 reading of a target at exactly 1.000 m (firmware assumes 343 m/s):",
           "  air [C]   c [m/s]   echo [ms]   reading [mm]   error [mm]   error [%]"]
    for t in (0.0, 15.0, 20.0, 25.0, 35.0, 45.0):
        reading = ultrasonic_reading_m(1.0, t)
        out.append(f"  {t:7.0f}  {speed_of_sound(t):8.1f}  {echo_time_s(1.0, t) * 1000:9.3f}"
                   f"  {reading * 1000:12.1f}  {(reading - 1.0) * 1000:10.1f}  {(reading - 1.0) * 100:9.2f}")
    out.append("\nBeam footprint radius (15 deg US-100 cone vs 27 deg VL53L1X full FoV vs 4x4 ROI ~ 6.8 deg):")
    out.append("  distance [m]   US-100 [cm]   VL53L1X [cm]   VL53L1X 4x4 ROI [cm]")
    for d in (0.25, 0.5, 1.0, 2.0, 3.0):
        out.append(f"  {d:12.2f}  {beam_footprint_radius_m(d, 15) * 100:12.1f}  {beam_footprint_radius_m(d, 27) * 100:13.1f}"
                   f"  {beam_footprint_radius_m(d, vl53l1x_roi_fov_deg(4)) * 100:20.1f}")
    out.append(f"\nMax ping rate for a 4.5 m range at 20 C (1.5x decay margin): {max_ping_rate_hz(4.5):.0f} Hz")
    out.append(f"Blind zone for 120 us of transducer ring-down: {blind_zone_m(120e-6) * 100:.1f} cm")
    out.append(f"\nLight: round trip for 1 mm = {light_round_trip_s(0.001) * 1e12:.2f} ps;"
               f" for 4 m = {light_round_trip_s(4.0) * 1e9:.2f} ns")
    out.append("Relative return signal and shot-noise sigma vs distance (1 m = 1.0):")
    out.append("  distance [m]   signal   sigma (dark)   sigma (ambient = 3x signal at 1 m)")
    for d in (0.5, 1.0, 2.0, 3.0, 4.0):
        out.append(f"  {d:12.1f}  {relative_return_signal(d):7.3f}  {shot_noise_sigma_scale(d):13.2f}"
                   f"  {shot_noise_sigma_scale(d, ambient_fraction=3.0):35.2f}")
    return "\n".join(out)


if __name__ == "__main__":
    print(tables())
