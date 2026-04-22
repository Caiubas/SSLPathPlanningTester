"""
trajectory_controller.py
=========================
Trajectory-following controller for SSL robots.

Given a reference trajectory — a sequence of (position, expected_velocity)
waypoints — this module computes a velocity command that keeps the robot on the
path using:

  1. **Orthogonal projection** of the robot onto the current path segment to
     decompose the positioning error into:
       - *cross-track error* (e_ct)  — perpendicular distance to the segment
       - *along-track error* (e_at)  — signed lag/lead along the segment
  2. **PID correction** in the path frame (separate gains per axis).
  3. **Velocity feedforward** — the reference velocity at the projected point
     (interpolated between waypoints) is added directly to the PID output so
     the robot tracks a moving reference even with small gains.
  4. **Waypoint advancement** — a waypoint is considered reached when the
     robot is within a configurable radius *and* the projection has passed
     the waypoint (projection parameter t ≥ 1).  This prevents the robot
     from skipping waypoints on curved paths.

Output is a world-frame velocity Vector clamped to vmax.  Convert to robot
frame with ``_world_vel_to_robot_frame`` from planner_handler as usual.

Typical usage
-------------
::

    from trajectory_controller import TrajectoryController, TrajectoryPoint

    traj = [
        TrajectoryPoint(Point(0.0, 0.0), Vector(1.0, 0.0)),
        TrajectoryPoint(Point(1.0, 0.5), Vector(1.0, 0.0)),
        TrajectoryPoint(Point(2.0, 0.0), Vector(0.0, 0.0)),
    ]

    ctrl = TrajectoryController(
        kp_ct=2.0, ki_ct=0.05, kd_ct=0.1,
        kp_at=1.5, ki_at=0.02, kd_at=0.08,
        vmax=3.0,
        waypoint_radius=0.08,
        feedforward_weight=1.0,
    )
    ctrl.set_trajectory(traj)

    # inside the control loop (called at each vision tick):
    vel: Vector = ctrl.update(current_pos, now=time.monotonic())
    if ctrl.finished:
        ...  # trajectory complete
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import NamedTuple

from pathplan.main import Point, Vector
from pid_controller import PIDController1D


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class TrajectoryPoint(NamedTuple):
    """A single waypoint in the reference trajectory."""
    position: Point          # world-frame position  (metres)
    velocity: Vector         # expected world-frame velocity at this point (m/s)


@dataclass
class ProjectionResult:
    """
    Result of projecting a robot position onto a path segment.

    Attributes
    ----------
    foot : Point
        The closest point on the *infinite* line through the segment.
    t : float
        Normalised parameter along the segment [0, 1].
        t < 0  → foot is before the segment start.
        t > 1  → foot is past the segment end.
    e_ct : float
        Cross-track error — signed perpendicular distance to the segment.
        Positive when the robot is to the *left* of the direction of travel.
    e_at : float
        Along-track error — signed lag along the segment.
        Positive when the robot is *behind* the reference point on the segment.
    ref_velocity : Vector
        Reference velocity linearly interpolated at the projected point.
    segment_dir : Vector
        Unit vector along the segment (start → end).
    """
    foot: Point
    t: float
    e_ct: float
    e_at: float
    ref_velocity: Vector
    segment_dir: Vector


@dataclass
class ControllerDiagnostics:
    """Snapshot of internal state for logging / visualisation."""
    waypoint_index: int = 0
    projection: ProjectionResult | None = None
    pid_ct_output: float = 0.0
    pid_at_output: float = 0.0
    feedforward: Vector = field(default_factory=lambda: Vector(0.0, 0.0))
    command_before_clamp: Vector = field(default_factory=lambda: Vector(0.0, 0.0))
    command: Vector = field(default_factory=lambda: Vector(0.0, 0.0))
    finished: bool = False


# ---------------------------------------------------------------------------
# Geometry helpers (kept local to avoid polluting namespace)
# ---------------------------------------------------------------------------

def _dot(a: Vector, b: Vector) -> float:
    return a.x * b.x + a.y * b.y


def _norm(v: Vector) -> float:
    return math.hypot(v.x, v.y)


def _unit(v: Vector) -> Vector:
    n = _norm(v)
    if n < 1e-12:
        return Vector(0.0, 0.0)
    return Vector(v.x / n, v.y / n)


def _sub(a: Point, b: Point) -> Vector:
    """Vector from point b to point a  (a − b)."""
    return Vector(a.x - b.x, a.y - b.y)


def _lerp_velocity(v0: Vector, v1: Vector, t: float) -> Vector:
    """Linear interpolation between two velocity vectors."""
    t = max(0.0, min(1.0, t))
    return Vector(v0.x + t * (v1.x - v0.x), v0.y + t * (v1.y - v0.y))


def _project_point_onto_segment(
    robot: Point,
    seg_start: Point,
    seg_end: Point,
    v_start: Vector,
    v_end: Vector,
) -> ProjectionResult:
    """
    Project *robot* onto the segment [seg_start, seg_end].

    Cross-track error
    -----------------
    Computed as the signed perpendicular distance from *robot* to the infinite
    line through the segment.  Sign convention: positive when the robot is to
    the left of the direction of travel (right-hand rule, CCW positive).

    Along-track error
    -----------------
    The reference point on the segment is the foot of the perpendicular (clamped
    to [0, 1]).  The along-track error is the distance from *robot* to the foot,
    projected onto the segment direction — positive when the robot is *behind*
    the foot (has not yet reached the reference point).
    """
    seg_vec = _sub(seg_end, seg_start)       # Vector along segment
    seg_len = _norm(seg_vec)

    if seg_len < 1e-9:
        # Degenerate segment — both endpoints coincide
        return ProjectionResult(
            foot=seg_start,
            t=0.0,
            e_ct=math.hypot(robot.x - seg_start.x, robot.y - seg_start.y),
            e_at=0.0,
            ref_velocity=v_start,
            segment_dir=Vector(1.0, 0.0),
        )

    seg_dir = _unit(seg_vec)                 # unit vector along segment
    # Perpendicular unit vector (left of travel, CCW)
    seg_perp = Vector(-seg_dir.y, seg_dir.x)

    robot_vec = _sub(robot, seg_start)

    # Scalar projection onto segment → parameter t
    t = _dot(robot_vec, seg_dir) / seg_len   # unbounded
    t_clamped = max(0.0, min(1.0, t))

    foot = Point(
        seg_start.x + t_clamped * seg_vec.x,
        seg_start.y + t_clamped * seg_vec.y,
    )

    # Cross-track error: signed distance on the perpendicular axis
    e_ct = _dot(robot_vec, seg_perp)         # positive → robot left of path

    # Along-track error: how far behind the robot is relative to the foot
    # (foot is at parameter t_clamped; robot projection is at t)
    # Positive → robot has NOT yet reached the foot
    foot_to_robot_along = _dot(_sub(robot, foot), seg_dir)
    e_at = -foot_to_robot_along              # negate: positive = robot behind

    ref_vel = _lerp_velocity(v_start, v_end, t_clamped)

    return ProjectionResult(
        foot=foot,
        t=t,
        e_ct=e_ct,
        e_at=e_at,
        ref_velocity=ref_vel,
        segment_dir=seg_dir,
    )


# ---------------------------------------------------------------------------
# Main controller
# ---------------------------------------------------------------------------

class TrajectoryController:
    """
    PID-based trajectory follower with orthogonal projection and feedforward.

    Parameters
    ----------
    kp_ct, ki_ct, kd_ct : float
        PID gains for the *cross-track* axis (perpendicular to path).
    kp_at, ki_at, kd_at : float
        PID gains for the *along-track* axis (parallel to path).
    vmax : float
        Maximum resultant speed of the output command (m/s).
    waypoint_radius : float
        Distance threshold to consider a waypoint reached (metres).
        The waypoint is only advanced when *both* this condition is met
        *and* the projection parameter t ≥ 1 − ``waypoint_t_margin``.
    waypoint_t_margin : float
        Tolerance on the projection parameter t for waypoint advancement.
        Default 0.05 (i.e. advance when t ≥ 0.95).
    feedforward_weight : float
        Scale factor on the reference velocity feedforward term [0, 1].
        0 → pure PID feedback; 1 → full feedforward + PID correction.
    integral_limit : float
        Anti-windup clamp for both PID integrators (m/s equivalent).
    derivative_alpha : float
        Low-pass coefficient for the derivative filter (0 < α ≤ 1).
    """

    def __init__(
        self,
        # Cross-track gains
        kp_ct: float = 2.0,
        ki_ct: float = 0.05,
        kd_ct: float = 0.10,
        # Along-track gains
        kp_at: float = 1.5,
        ki_at: float = 0.02,
        kd_at: float = 0.08,
        # Limits
        vmax: float = 3.0,
        waypoint_radius: float = 0.08,
        waypoint_t_margin: float = 0.05,
        # Feedforward
        feedforward_weight: float = 1.0,
        # PID internals
        integral_limit: float = 1.0,
        derivative_alpha: float = 0.8,
    ) -> None:

        self.vmax = vmax
        self.waypoint_radius = waypoint_radius
        self.waypoint_t_margin = waypoint_t_margin
        self.feedforward_weight = feedforward_weight

        # Separate PIDs for cross-track and along-track axes
        self._pid_ct = PIDController1D(
            kp_ct, ki_ct, kd_ct,
            output_limit=vmax,
            integral_limit=integral_limit,
            derivative_alpha=derivative_alpha,
        )
        self._pid_at = PIDController1D(
            kp_at, ki_at, kd_at,
            output_limit=vmax,
            integral_limit=integral_limit,
            derivative_alpha=derivative_alpha,
        )

        # Trajectory state
        self._trajectory: list[TrajectoryPoint] = []
        self._wp_index: int = 0          # index of the *end* waypoint of current segment
        self._finished: bool = True

        # Diagnostics (updated each call to update())
        self.diagnostics = ControllerDiagnostics()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def finished(self) -> bool:
        """True when the robot has reached the last waypoint."""
        return self._finished

    @property
    def waypoint_index(self) -> int:
        """Index of the end-waypoint of the segment currently being tracked."""
        return self._wp_index

    def set_trajectory(self, trajectory: list[TrajectoryPoint]) -> None:
        """
        Load a new reference trajectory and reset all controller state.

        Parameters
        ----------
        trajectory :
            Ordered list of ``TrajectoryPoint``.  Must have at least 2 points.
            The last waypoint should have ``velocity = Vector(0, 0)`` so the
            robot comes to rest.

        Raises
        ------
        ValueError
            If *trajectory* has fewer than 2 points.
        """
        if len(trajectory) < 2:
            raise ValueError(
                "Trajectory must have at least 2 waypoints "
                f"(got {len(trajectory)})."
            )
        self._trajectory = list(trajectory)
        self._wp_index = 1      # first segment: [0] → [1]
        self._finished = False
        self._pid_ct.reset()
        self._pid_at.reset()
        self.diagnostics = ControllerDiagnostics()

    def reset(self) -> None:
        """Reset PID state without changing the trajectory (e.g. after a pause)."""
        self._pid_ct.reset()
        self._pid_at.reset()

    # ------------------------------------------------------------------
    # Main update — called once per control tick
    # ------------------------------------------------------------------

    def update(
        self,
        robot_pos: Point,
        now: float | None = None,
    ) -> Vector:
        """
        Compute the velocity command for the current tick.

        Parameters
        ----------
        robot_pos :
            Current robot position in the world frame (metres).
        now :
            Timestamp in seconds.  Defaults to ``time.monotonic()``.

        Returns
        -------
        Vector
            World-frame velocity command (m/s), magnitude ≤ ``vmax``.
            Returns ``Vector(0, 0)`` if the trajectory is finished or not set.
        """
        if now is None:
            now = time.monotonic()

        zero = Vector(0.0, 0.0)

        if self._finished or len(self._trajectory) < 2:
            self.diagnostics.finished = True
            self.diagnostics.command = zero
            return zero

        # ------------------------------------------------------------------
        # 1. Waypoint advancement check
        # ------------------------------------------------------------------
        self._maybe_advance_waypoint(robot_pos)

        if self._finished:
            self.diagnostics.finished = True
            self.diagnostics.command = zero
            return zero

        # ------------------------------------------------------------------
        # 2. Orthogonal projection onto the current segment
        # ------------------------------------------------------------------
        seg_start = self._trajectory[self._wp_index - 1]
        seg_end   = self._trajectory[self._wp_index]

        proj = _project_point_onto_segment(
            robot_pos,
            seg_start.position,
            seg_end.position,
            seg_start.velocity,
            seg_end.velocity,
        )

        # ------------------------------------------------------------------
        # 3. PID in path frame
        # ------------------------------------------------------------------
        # Cross-track: drive e_ct → 0  (push robot back onto segment)
        # The correction must be applied perpendicular to the segment.
        ct_correction_scalar = self._pid_ct.update(-proj.e_ct, now)

        # Along-track: drive e_at → 0  (close the lag/lead along the segment)
        at_correction_scalar = self._pid_at.update(-proj.e_at, now)

        # PID outputs live in the path frame; rotate back to world frame
        seg_dir  = proj.segment_dir
        seg_perp = Vector(-seg_dir.y, seg_dir.x)  # left of travel

        pid_world = Vector(
            at_correction_scalar * seg_dir.x  + ct_correction_scalar * seg_perp.x,
            at_correction_scalar * seg_dir.y  + ct_correction_scalar * seg_perp.y,
        )

        # ------------------------------------------------------------------
        # 4. Velocity feedforward
        #    Use the reference velocity interpolated at the projected foot.
        # ------------------------------------------------------------------
        ff = Vector(
            self.feedforward_weight * proj.ref_velocity.x,
            self.feedforward_weight * proj.ref_velocity.y,
        )

        # ------------------------------------------------------------------
        # 5. Combine and clamp to vmax
        # ------------------------------------------------------------------
        raw = Vector(pid_world.x + ff.x, pid_world.y + ff.y)
        cmd = self._clamp_to_vmax(raw)

        # ------------------------------------------------------------------
        # 6. Update diagnostics
        # ------------------------------------------------------------------
        self.diagnostics.waypoint_index      = self._wp_index
        self.diagnostics.projection          = proj
        self.diagnostics.pid_ct_output       = ct_correction_scalar
        self.diagnostics.pid_at_output       = at_correction_scalar
        self.diagnostics.feedforward         = ff
        self.diagnostics.command_before_clamp = raw
        self.diagnostics.command             = cmd
        self.diagnostics.finished            = self._finished

        return cmd

    # ------------------------------------------------------------------
    # Waypoint logic
    # ------------------------------------------------------------------

    def _maybe_advance_waypoint(self, robot_pos: Point) -> None:
        """
        Advance ``_wp_index`` while the robot has reached the current waypoint.

        A waypoint is considered reached when:
          (a) Euclidean distance ≤ ``waypoint_radius``, *or*
          (b) The projection parameter t of the robot onto the segment
              satisfies t ≥ 1 − ``waypoint_t_margin``  AND the robot has
              crossed the waypoint (distance condition is met within 2× radius).

        Condition (b) prevents stalling on curved paths where the robot may
        never get within ``waypoint_radius`` of an intermediate waypoint
        due to the path geometry.

        When the last waypoint is reached, ``_finished`` is set to True.
        """
        while not self._finished:
            seg_start = self._trajectory[self._wp_index - 1]
            seg_end   = self._trajectory[self._wp_index]

            dist_to_end = math.hypot(
                robot_pos.x - seg_end.position.x,
                robot_pos.y - seg_end.position.y,
            )

            # Condition (a): close enough to the waypoint
            reached_by_dist = dist_to_end <= self.waypoint_radius

            # Condition (b): projection has passed the waypoint
            proj = _project_point_onto_segment(
                robot_pos,
                seg_start.position,
                seg_end.position,
                seg_start.velocity,
                seg_end.velocity,
            )
            reached_by_proj = (
                proj.t >= 1.0 - self.waypoint_t_margin
                and dist_to_end <= 2.0 * self.waypoint_radius
            )

            if not (reached_by_dist or reached_by_proj):
                break   # waypoint not yet reached — keep current segment

            # Waypoint reached
            is_last = self._wp_index == len(self._trajectory) - 1
            if is_last:
                self._finished = True
                self._pid_ct.reset()
                self._pid_at.reset()
            else:
                self._wp_index += 1
                # Reset derivative / integral to avoid transient on new segment
                self._pid_ct.reset()
                self._pid_at.reset()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _clamp_to_vmax(self, v: Vector) -> Vector:
        """Clamp vector magnitude to vmax while preserving direction."""
        speed = math.hypot(v.x, v.y)
        if speed > self.vmax and speed > 0.0:
            scale = self.vmax / speed
            return Vector(v.x * scale, v.y * scale)
        return v

    # ------------------------------------------------------------------
    # Convenience: build trajectory from a plain path + speed profile
    # ------------------------------------------------------------------

    @staticmethod
    def from_path(
        path: list[Point],
        cruise_speed: float = 1.5,
        arrival_speed: float = 0.0,
    ) -> "list[TrajectoryPoint]":
        """
        Build a ``list[TrajectoryPoint]`` from a bare list of ``Point``.

        Each intermediate waypoint gets a velocity directed toward the next
        point with magnitude ``cruise_speed``.  The final waypoint gets
        ``arrival_speed`` (defaults to zero so the robot stops).

        Parameters
        ----------
        path :
            Waypoints in world frame (metres).  At least 2 required.
        cruise_speed :
            Speed to assign to every intermediate waypoint (m/s).
        arrival_speed :
            Speed at the last waypoint (m/s).  Usually 0.

        Returns
        -------
        list[TrajectoryPoint]
            Ready to pass to ``TrajectoryController.set_trajectory()``.

        Example
        -------
        ::

            traj = TrajectoryController.from_path(
                planner.plan(start, goal),
                cruise_speed=2.0,
            )
            ctrl.set_trajectory(traj)
        """
        if len(path) < 2:
            raise ValueError("Path must have at least 2 points.")

        result: list[TrajectoryPoint] = []
        n = len(path)
        for i, pt in enumerate(path):
            if i < n - 1:
                # Direction toward next point
                dx = path[i + 1].x - pt.x
                dy = path[i + 1].y - pt.y
                dist = math.hypot(dx, dy)
                if dist > 1e-9:
                    speed = cruise_speed if i < n - 2 else arrival_speed
                    vel = Vector(speed * dx / dist, speed * dy / dist)
                else:
                    vel = Vector(0.0, 0.0)
            else:
                # Last point
                vel = Vector(0.0, 0.0)
            result.append(TrajectoryPoint(position=pt, velocity=vel))

        return result