#!/usr/bin/env python3
"""
LONGITUDINAL SPEED PID CONTROLLER
ROS2 Humble | Python 3.10+
University Final Project - ADAS

A textbook PID controller used to keep the ego vehicle at a target speed.

How it works (the question the controller asks every step):

    What speed do I want?     -> target_speed
    What speed am I at now?   -> current_speed
    The difference is:        -> error = target_speed - current_speed

From that error it produces a command (an acceleration in m/s²):

    P (proportional): react to the error *right now*       Kp · error
    I (integral):     react to error built up *over time*  Ki · Σ error·dt
    D (derivative):   react to how fast the error changes  Kd · Δerror/dt

    output = P + I + D

A positive output means "give gas" (accelerate); a negative output means
"ease off / brake gently" to approach the target.
"""

from dataclasses import dataclass


@dataclass
class PIDDebug:
    """Per-step breakdown of the controller, handy for dashboards/plots."""
    error: float = 0.0
    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    output: float = 0.0


class PIDController:
    """
    Generic single-input PID controller with output clamping and
    integral anti-windup.

    Example:
        pid = PIDController(kp=0.9, ki=0.25, kd=0.05,
                            output_min=-2.0, output_max=2.0)
        accel_cmd = pid.update(target=13.9, current=11.1, dt=0.02)
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float, output_max: float,
                 integral_limit: float = 1e9):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        # Internal state carried between steps
        self.integral = 0.0
        self.prev_error = 0.0
        self.debug = PIDDebug()

    def reset(self):
        """Clear accumulated state. Call when control is handed off
        (e.g. while emergency braking) to avoid integral wind-up."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.debug = PIDDebug()

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def update(self, target: float, current: float, dt: float) -> float:
        """
        Run one control step and return the clamped output command.

        target  : desired value (target speed, m/s)
        current : measured value (current speed, m/s)
        dt      : time elapsed since the previous step (s)
        """
        if dt <= 0.0:
            return self.debug.output

        # 1. The error: how far we are from the target
        error = target - current

        # 2. Proportional term — present error
        p_term = self.kp * error

        # 3. Integral term — accumulated past error (with anti-windup clamp)
        self.integral += error * dt
        self.integral = self._clamp(self.integral,
                                    -self.integral_limit,
                                    self.integral_limit)
        i_term = self.ki * self.integral

        # 4. Derivative term — rate of change of the error
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        self.prev_error = error

        # 5. Sum and clamp to the actuator limits
        output = self._clamp(p_term + i_term + d_term,
                             self.output_min, self.output_max)

        # Record breakdown for telemetry / visualization
        self.debug = PIDDebug(error=error, p=p_term, i=i_term,
                              d=d_term, output=output)
        return output
