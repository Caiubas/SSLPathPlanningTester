"""
pid_controller.py
=================
PID controller for SSL robot motion control.

Provides:
  - PIDController1D  — single-axis PID (position or velocity)
  - PIDController2D  — coupled 2-D PID that returns a velocity Vector
  - AngularPIDController — wraps angle error to [-π, π] for heading control

Design notes
------------
* All units are SI: metres, metres/second, radians, seconds.
* Derivative filtering uses an exponential low-pass (alpha parameter) to
  suppress high-frequency noise from vision jitter.
* Integral windup is clamped via ``integral_limit``.
* ``reset()`` should be called whenever a robot acquires a new target so
  that stale integral/derivative state does not cause a transient spike.
* The 2-D controller clamps the output magnitude to ``output_limit`` while
  preserving direction — this keeps the commanded velocity inside VMAX.
"""

from __future__ import annotations

import math
import time

from pathplan.main import Vector, Point


# ---------------------------------------------------------------------------
# 1-D PID
# ---------------------------------------------------------------------------

class PIDController1D:
    """
    Discrete PID controller for a single axis.

    Parameters
    ----------
    kp, ki, kd:
        Proportional, integral and derivative gains.
    output_limit:
        Symmetric clamp applied to the final output  (|u| ≤ output_limit).
        Set to ``float('inf')`` to disable.
    integral_limit:
        Symmetric clamp on the running integral term to prevent windup.
        Set to ``float('inf')`` to disable.
    derivative_alpha:
        Low-pass filter coefficient for the derivative (0 < alpha ≤ 1).
        alpha = 1  → no filtering (raw finite difference).
        alpha → 0  → very heavy filtering (slow derivative response).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = float("inf"),
        integral_limit: float = float("inf"),
        derivative_alpha: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.derivative_alpha = derivative_alpha

        self._integral: float = 0.0
        self._prev_error: float | None = None
        self._filtered_deriv: float = 0.0
        self._last_time: float | None = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear accumulated state (call on new target / re-enable)."""
        self._integral = 0.0
        self._prev_error = None
        self._filtered_deriv = 0.0
        self._last_time = None

    # ------------------------------------------------------------------
    def update(self, error: float, now: float | None = None) -> float:
        """
        Compute one PID step.

        Parameters
        ----------
        error:
            Signed scalar error  (setpoint − measurement).
        now:
            Timestamp in seconds.  Defaults to ``time.monotonic()``.

        Returns
        -------
        float
            Control output, clamped to ±``output_limit``.
        """
        if now is None:
            now = time.monotonic()

        # --- time delta -----------------------------------------------
        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time
            if dt <= 0.0:
                dt = 0.0
        self._last_time = now

        # --- proportional ---------------------------------------------
        p = self.kp * error

        # --- integral (with anti-windup clamp) -----------------------
        if dt > 0.0:
            self._integral += error * dt
            self._integral = max(
                -self.integral_limit, min(self.integral_limit, self._integral)
            )
        i = self.ki * self._integral

        # --- derivative (filtered finite difference) -----------------
        if dt > 0.0 and self._prev_error is not None:
            raw_deriv = (error - self._prev_error) / dt
            alpha = self.derivative_alpha
            self._filtered_deriv = (
                alpha * raw_deriv + (1.0 - alpha) * self._filtered_deriv
            )
        d = self.kd * self._filtered_deriv
        self._prev_error = error

        # --- output clamp --------------------------------------------
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    # ------------------------------------------------------------------
    @property
    def integral(self) -> float:
        """Read-only access to the current integral accumulator."""
        return self._integral


# ---------------------------------------------------------------------------
# 2-D PID  (position → velocity command)
# ---------------------------------------------------------------------------

class PIDController2D:
    """
    Coupled 2-D PID for position control in the world frame.

    Internally uses two independent ``PIDController1D`` instances (one per
    axis) and then clamps the resulting velocity vector to ``output_limit``
    while preserving its direction.

    Typical usage in ``_plan_and_control``::

        pid = PIDController2D(kp=1.5, ki=0.0, kd=0.1, output_limit=VMAX)
        ...
        vel: Vector = pid.update(current_pos, target_pos)

    Parameters
    ----------
    kp, ki, kd:
        Gains shared by both axes.
    output_limit:
        Maximum resultant speed (metres/second).  Defaults to ``VMAX``.
    integral_limit:
        Per-axis integral windup clamp.
    derivative_alpha:
        Low-pass coefficient for derivative filtering.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 3.0,
        integral_limit: float = 1.0,
        derivative_alpha: float = 0.8,
    ) -> None:
        self.output_limit = output_limit
        self._x = PIDController1D(kp, ki, kd, float("inf"), integral_limit, derivative_alpha)
        self._y = PIDController1D(kp, ki, kd, float("inf"), integral_limit, derivative_alpha)

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Reset both axes (call whenever a new target is assigned)."""
        self._x.reset()
        self._y.reset()

    # ------------------------------------------------------------------
    def update(
        self,
        current: Point,
        target: Point,
        now: float | None = None,
    ) -> Vector:
        """
        Compute a velocity command in the world frame.

        Parameters
        ----------
        current:
            Robot's current position (metres).
        target:
            Desired position (metres).
        now:
            Timestamp in seconds.  Defaults to ``time.monotonic()``.

        Returns
        -------
        Vector
            Velocity command (m/s), magnitude ≤ ``output_limit``.
        """
        if now is None:
            now = time.monotonic()

        vx = self._x.update(target.x - current.x, now)
        vy = self._y.update(target.y - current.y, now)

        # Clamp to circular speed limit while preserving direction
        speed = math.hypot(vx, vy)
        if speed > self.output_limit and speed > 0.0:
            scale = self.output_limit / speed
            vx *= scale
            vy *= scale

        return Vector(vx, vy)

    # ------------------------------------------------------------------
    @property
    def x_controller(self) -> PIDController1D:
        return self._x

    @property
    def y_controller(self) -> PIDController1D:
        return self._y


# ---------------------------------------------------------------------------
# Angular PID  (heading control)
# ---------------------------------------------------------------------------

class AngularPIDController:
    """
    PID controller for heading / orientation (radians).

    Wraps the angular error to ``[-π, π]`` before feeding it to the
    underlying 1-D PID so the robot always turns the short way.

    Parameters
    ----------
    kp, ki, kd:
        Gains.
    output_limit:
        Maximum angular velocity (rad/s).
    integral_limit:
        Anti-windup clamp (rad).
    derivative_alpha:
        Low-pass coefficient for derivative filtering.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = math.pi,
        integral_limit: float = math.pi / 2.0,
        derivative_alpha: float = 0.8,
    ) -> None:
        self._pid = PIDController1D(
            kp, ki, kd, output_limit, integral_limit, derivative_alpha
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap *angle* to the range (-π, π]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._pid.reset()

    # ------------------------------------------------------------------
    def update(
        self,
        current_angle: float,
        target_angle: float,
        now: float | None = None,
    ) -> float:
        """
        Compute an angular velocity command (rad/s).

        Parameters
        ----------
        current_angle:
            Robot's current orientation in radians (world frame).
        target_angle:
            Desired orientation in radians (world frame).
        now:
            Timestamp in seconds.

        Returns
        -------
        float
            Angular velocity command (rad/s), clamped to ±``output_limit``.
        """
        error = self._wrap(target_angle - current_angle)
        return self._pid.update(error, now)

    # ------------------------------------------------------------------
    @property
    def inner(self) -> PIDController1D:
        """The underlying ``PIDController1D`` (for diagnostics)."""
        return self._pid