"""08.06 — A full PID: derivative on the measurement, with a filtered derivative.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.06``.
Only the standard library is needed. Start from your 08.05 PIController.

    output = kp * error + integral + d_term          clamped to [output_min, output_max]

* integral: exactly as in 08.05 (stored multiplied by ki; conditional integration + clamping).
  The unclamped output used by conditional integration now includes d_term.
* d_term: derivative of the MEASUREMENT, not of the error, then a first-order low-pass filter:

      raw_rate  = (measured - last_measured) / dt
      alpha     = dt / (derivative_filter_tau_s + dt)          # tau = 0 -> alpha = 1 -> unfiltered
      rate     += alpha * (raw_rate - rate)                    # rate starts at 0
      d_term    = -kd * rate

  On the very first update (no last measurement yet) the rate stays 0.
  Why minus: error = setpoint - measured, so d(error)/dt = -d(measured)/dt whenever the setpoint
  is constant. When the setpoint jumps, d(error)/dt is a huge spike ("derivative kick");
  d(measured)/dt is not.
"""

from __future__ import annotations


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, derivative_filter_tau_s: float = 0.0,
                 output_min: float = -1.0, output_max: float = 1.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.derivative_filter_tau_s = derivative_filter_tau_s
        self.output_min = output_min
        self.output_max = output_max
        self.reset()

    def reset(self) -> None:
        """Clear all memory: integral, filtered rate, last measurement, d_term and output."""
        self.integral = 0.0
        self.d_term = 0.0  # the last derivative contribution, for logging
        self.output = 0.0
        # TODO(student): add whatever the derivative needs to remember (filtered rate, last measurement).

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        """One control step; returns the duty and stores it in ``self.output``."""
        # TODO(student):
        #   1. error and proportional term
        #   2. filtered derivative of the measurement -> self.d_term
        #   3. integral with anti-windup (08.05), counting d_term in the unclamped output
        #   4. output = P + integral + d_term, clamped
        raise NotImplementedError("PIDController.update")
