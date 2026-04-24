"""
planner_handler.py
==================
LCM-based motion planning handler for an SSL robot soccer system.

Subscribes to:
  - "vision"   → vision message (robots, ball, field)
  - "GC"       → game_controller message
  - "TARGETS"  → JSON bytes {"robot_id": int, "x": float, "y": float}

Publishes to:
  - "IA"       → ia message with per-robot velocity commands

Control architecture
--------------------
Path planning (PathPlanner / RRT) produces a list of waypoints.  At each
vision tick the **TrajectoryController** is used to compute a velocity
command by:

  1. Orthogonally projecting the robot onto the current path segment.
  2. Running separate PID loops for the cross-track and along-track errors.
  3. Adding a velocity feedforward term interpolated from the reference
     trajectory so the robot tracks a moving reference with small gains.
  4. Advancing the waypoint index when the robot is within *waypoint_radius*
     or when the projection parameter passes the waypoint.

The BangBangOptimizer is still available and used to build the reference
velocity profile attached to each path waypoint (TrajectoryPoint.velocity).
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass, field

import lcm

from data import vision, ia, game_controller, robot  # LCM generated types
from pathplan.main import Point, Vector, Circle, Quadrilateral
from pathplan.main import World, PathPlanner
from pathplan.rrt import RRT
from pathplan.new_bboptimizer import (
    State2D,
    PhaseState,
    AccelLimits,
    Steer2D,
    BangBangOptimizer,
)

from control import TrajectoryController, TrajectoryPoint
from pid_controller import PIDController1D  # noqa: F401  (re-exported by control)

# ---------------------------------------------------------------------------
# Motion constants
# ---------------------------------------------------------------------------
VMAX = 3.0*1.42
UMAX = Vector(2, 2)
UMIN = Vector(-2, -2)
ROBOT_RADIUS = 0.18        # metres
FIELD_MARGIN = 0.1         # metres inset from field boundary

# Tunable thresholds
TARGET_REACHED_DIST   = 0.05    # metres — stop if closer than this to final target
WAYPOINT_ADVANCE_DIST = 0.10    # metres — legacy; TrajectoryController uses its own
REPLAN_TIMEOUT        = 30    # seconds — force replan after this long
REPLAN_DEVIATION      = 0.5    # metres — replan if deviated from expected waypoint
PATH_PLANNER_MAX_ITER = 1000

# TrajectoryController gains — tune these for your field / robot dynamics
TRAJ_KP_CT            = 3    # cross-track proportional gain
TRAJ_KI_CT            = 0.05   # cross-track integral gain
TRAJ_KD_CT            = 0.10   # cross-track derivative gain
TRAJ_KP_AT            = 1    # along-track proportional gain
TRAJ_KI_AT            = 0.02   # along-track integral gain
TRAJ_KD_AT            = 0.08   # along-track derivative gain
TRAJ_WAYPOINT_RADIUS  = 0.08   # metres — waypoint reached threshold
TRAJ_FF_WEIGHT        = 1.0    # feedforward weight [0, 1]

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Per-robot state
# ---------------------------------------------------------------------------
@dataclass
class RobotState:
    robot_id: int
    current_pos: Point
    current_vel: Vector
    target_pos: Point | None = None
    current_path: list[Point] = field(default_factory=list)
    path_index: int = 0
    last_planned_at: float = 0.0

    # TrajectoryController instance — one per robot, persists across ticks
    traj_controller: TrajectoryController = field(
        default_factory=lambda: TrajectoryController(
            kp_ct=TRAJ_KP_CT, ki_ct=TRAJ_KI_CT, kd_ct=TRAJ_KD_CT,
            kp_at=TRAJ_KP_AT, ki_at=TRAJ_KI_AT, kd_at=TRAJ_KD_AT,
            vmax=VMAX,
            waypoint_radius=TRAJ_WAYPOINT_RADIUS,
            feedforward_weight=TRAJ_FF_WEIGHT,
        )
    )


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def _mm_to_m(val: float) -> float:
    return val / 1000.0


def _world_vel_to_robot_frame(vel_x: float, vel_y: float, orientation: float):
    """
    Rotate world-frame velocity (vel_x, vel_y) into the robot's local frame.

    The robot's forward (tangential) axis is defined by its orientation angle
    (radians, world frame).  Normal is 90° counter-clockwise from tangential.

    Returns (vel_tang, vel_normal).
    """
    cos_a = math.cos(orientation)
    sin_a = math.sin(orientation)
    vel_tang   =  vel_x * cos_a + vel_y * sin_a
    vel_normal = -vel_x * sin_a + vel_y * cos_a
    return vel_tang, vel_normal


def _build_field_boundary(field_msg, margin: float) -> Quadrilateral:
    """
    Build a Quadrilateral representing the inset field boundary.
    vision.field carries field_length and field_width in millimetres.
    """
    half_l = _mm_to_m(field_msg.field_length) / 2.0 - margin
    half_w = _mm_to_m(field_msg.field_width)  / 2.0 - margin
    return Quadrilateral([
        Point(-half_l, -half_w),
        Point( half_l, -half_w),
        Point( half_l,  half_w),
        Point(-half_l,  half_w),
    ])


def _build_trajectory(
    path: list[Point],
    current_vel: Vector,
    world: "World",
) -> list[TrajectoryPoint]:
    """
    Build a ``list[TrajectoryPoint]`` from a bare path produced by the planner.

    Tries to use BangBangOptimizer to attach reference velocities to each
    waypoint.  If the optimizer fails, or returns a state list of a different
    length than the path, falls back to a constant cruise-speed profile via
    ``TrajectoryController.from_path``.

    The last waypoint always gets ``velocity = Vector(0, 0)`` so the robot
    comes to rest.
    """
    if len(path) < 2:
        # Degenerate: single point — return a stopped waypoint
        return [TrajectoryPoint(position=path[0], velocity=Vector(0.0, 0.0))]

    try:
        limits = AccelLimits(UMIN, UMAX, vmax=VMAX)
        steer  = Steer2D(limits)

        x0  = State2D(PhaseState(path[0].x, current_vel.x),
                      PhaseState(path[0].y, current_vel.y))
        seq = steer.steer_list(path, current_vel)

        from pathplan.main import new_no_collision  # type: ignore
        opt    = BangBangOptimizer(steer, new_no_collision, world, max_iter = 4000)
        result = opt.optimize(x0, seq)
        states = result.integrate_list(x0)

        # states has one entry per time-step, not per waypoint.
        # We only use it if the count happens to match the path length exactly.
        if states:

            traj = [
                TrajectoryPoint(position=Point(s.x.q, s.y.q), velocity=Vector(s.x.v, s.y.v))
                for s in states
            ]
            path = [Point(s.x.q, s.y.q) for s in states]

            return traj

    except Exception as exc:
        log.debug("BangBang trajectory build failed (%s); using cruise profile.", exc)

    # Reliable fallback: cruise speed toward each next waypoint, stop at end.
    print("error on optimizer")
    return TrajectoryController.from_path(path, cruise_speed=min(VMAX * 0.5, 1.5))


# ---------------------------------------------------------------------------
# Main handler
# ---------------------------------------------------------------------------

class PlannerHandler:
    """Integrates vision/GC/target LCM messages and publishes ia commands."""

    def __init__(self, team_blue: bool = False):
        self.team_blue = team_blue

        # LCM setup
        self.lc = lcm.LCM()
        self.lc.subscribe("vision",   self._handle_vision)
        self.lc.subscribe("GC",       self._handle_gc)
        self.lc.subscribe("TARGETS",  self._handle_targets)

        # Per-robot state: robot_id → RobotState
        self.robot_states: dict[int, RobotState] = {}

        # Field boundary (set on first vision message)
        self.field_boundary: Quadrilateral | None = None

        log.info("PlannerHandler initialised (team_blue=%s)", team_blue)

    # ------------------------------------------------------------------
    # LCM callbacks
    # ------------------------------------------------------------------

    def _handle_gc(self, channel: str, data: bytes) -> None:
        try:
            msg = game_controller.decode(data)
            self.team_blue = bool(msg.team_blue)
            log.debug("GC update: team_blue=%s", self.team_blue)
        except Exception:
            log.warning("Failed to decode GC message", exc_info=True)

    def _handle_targets(self, channel: str, data: bytes) -> None:
        try:
            payload = json.loads(data)
            rid = int(payload["robot_id"])
            x   = float(payload["x"])
            y   = float(payload["y"])
            if rid not in self.robot_states:
                log.warning("Received target for unknown robot_id=%d; ignoring.", rid)
                return
            new_target = Point(x, y)
            state = self.robot_states[rid]

            # Only reset trajectory if the target actually changed
            if state.target_pos is None or (
                abs(state.target_pos.x - x) > 1e-4 or
                abs(state.target_pos.y - y) > 1e-4
            ):
                state.target_pos   = new_target
                state.current_path = []          # trigger replan
                state.traj_controller.reset()
                log.debug("Target set for robot %d → (%.3f, %.3f)", rid, x, y)
        except Exception:
            log.warning("Failed to parse TARGETS message", exc_info=True)

    def _handle_vision(self, channel: str, data: bytes) -> None:
        try:
            msg = vision.decode(data)
        except Exception:
            log.warning("Failed to decode vision message", exc_info=True)
            return

        # 1. Update field boundary
        if msg.field is not None:
            self.field_boundary = _build_field_boundary(msg.field, FIELD_MARGIN)

        if self.field_boundary is None:
            log.debug("Field boundary not yet known; skipping tick.")
            return

        # 2. Select own and opposing robot lists
        if self.team_blue:
            own_robots = msg.robots_blue
            opp_robots = msg.robots_yellow
        else:
            own_robots = msg.robots_yellow
            opp_robots = msg.robots_blue

        # 3. Upsert own robot states
        for r in own_robots:
            pos_m = Point(_mm_to_m(r.position_x), _mm_to_m(r.position_y))
            if r.robot_id in self.robot_states:
                self.robot_states[r.robot_id].current_pos = pos_m
            else:
                self.robot_states[r.robot_id] = RobotState(
                    robot_id=r.robot_id,
                    current_pos=pos_m,
                    current_vel=Vector(0.0, 0.0),
                )
            log.debug("Robot %d pos=(%.3f, %.3f)", r.robot_id, pos_m.x, pos_m.y)

        # 4. Build opposing-team obstacle list (constant for this tick)
        opp_obstacles = [
            Circle(
                center=Point(_mm_to_m(r.position_x), _mm_to_m(r.position_y)),
                radius=ROBOT_RADIUS,
            )
            for r in opp_robots
        ]

        # 5. Build ia output message
        msg_ia       = ia()
        msg_ia.timestamp = msg.timestamp
        ia_slots: dict[int, robot] = {}

        # 6. Plan & control for each own robot
        for r_vis in own_robots:
            rid = r_vis.robot_id
            state = self.robot_states[rid]

            # Prepare a zero-velocity ia slot (safe default)
            slot = robot()
            slot.id           = rid
            slot.spinner      = 0
            slot.kick         = 0
            slot.vel_tang     = 0.0
            slot.vel_normal   = 0.0
            slot.vel_ang      = 0.0
            slot.kick_speed_x = 0.0
            slot.kick_speed_z = 0.0
            ia_slots[rid] = slot

            try:
                self._plan_and_control(state, r_vis, opp_obstacles, own_robots, slot)
            except Exception:
                log.warning(
                    "Planning failed for robot %d; publishing zero velocities.", rid,
                    exc_info=True,
                )

        # 7. Assemble ia message
        msg_ia.robots_size = len(ia_slots)
        msg_ia.robots = [ia_slots[rid] for rid in sorted(ia_slots)]

        # 8. Publish
        self.lc.publish("IA", msg_ia.encode())
        log.debug("Published IA for %d robots", msg_ia.robots_size)

    # ------------------------------------------------------------------
    # Per-robot planning & control
    # ------------------------------------------------------------------

    def _plan_and_control(
        self,
        state: RobotState,
        r_vis,
        opp_obstacles: list,
        own_robots: list,
        slot: robot,
    ) -> None:
        rid = state.robot_id
        pos = state.current_pos

        # a. No target → zero velocity
        if state.target_pos is None:
            log.debug("Robot %d has no target.", rid)
            return

        target = state.target_pos

        # b. Close enough to final target → clear target, zero velocity
        if pos.distance_to(target) < TARGET_REACHED_DIST:
            log.info("Robot %d reached target (%.3f, %.3f).", rid, target.x, target.y)
            state.target_pos   = None
            state.current_path = []
            state.current_vel  = Vector(0.0, 0.0)
            state.traj_controller.reset()
            return

        # Build per-robot obstacle list: all own robots except self + opposing
        own_obstacles = [
            Circle(
                center=Point(_mm_to_m(r.position_x), _mm_to_m(r.position_y)),
                radius=ROBOT_RADIUS,
            )
            for r in own_robots
            if r.robot_id != rid
        ]
        obstacles = opp_obstacles + own_obstacles
        world = World(obstacles=obstacles, boundaries=self.field_boundary)

        # c. Replan condition
        now = time.time()
        need_replan = False

        if not state.current_path:
            need_replan = True
            print("REPLAN EMPTY")
            log.debug("Robot %d: no path, replanning.", rid)
        elif now - state.last_planned_at > REPLAN_TIMEOUT:
            need_replan = True
            print("REPLAN TIMEOUT")
            log.debug("Robot %d: path stale (%.1fs), replanning.", rid,
                      now - state.last_planned_at)
        else:
            # Check deviation from expected waypoint
            expected_wp = state.current_path[
                min(state.path_index, len(state.current_path) - 1)
            ]
            if pos.distance_to(expected_wp) > REPLAN_DEVIATION:
                need_replan = True
                print("REPLAN TIMEOUT")
                log.debug(
                    "Robot %d: deviated %.3fm from waypoint, replanning.",
                    rid, pos.distance_to(expected_wp),
                )

        if need_replan:
            self._replan(state, pos, target, world)
            # After replanning, load a fresh trajectory into the controller
            if state.current_path:
                traj = _build_trajectory(state.current_path, state.current_vel, world)
                state.traj_controller.set_trajectory(traj)
                log.debug("Robot %d: trajectory loaded (%d pts).", rid, len(traj))

        if not state.current_path:
            log.warning("Robot %d: path empty after replan; zero velocity.", rid)
            return

        # d. TrajectoryController is finished but we still have a path →
        #    the robot drifted; force a replan on the next tick.
        if state.traj_controller.finished:
            state.current_path = []
            log.debug("Robot %d: TrajectoryController finished; queuing replan.", rid)
            return

        # e. Compute velocity command via PID trajectory follower
        #    (includes cross-track PID + along-track PID + velocity feedforward)
        cmd: Vector = state.traj_controller.update(pos, now=time.monotonic())

        # Update estimated velocity from the command (used for the next
        # BangBang trajectory build)
        state.current_vel = cmd

        # f. Sync the legacy path_index with the TrajectoryController so the
        #    replan deviation check above stays meaningful.
        state.path_index = min(
            state.traj_controller.waypoint_index,
            len(state.current_path) - 1,
        )

        # g. Convert world-frame velocity to robot frame and fill the ia slot
        vel_tang, vel_normal = _world_vel_to_robot_frame(
            cmd.x, cmd.y, r_vis.orientation
        )
        slot.vel_tang   = vel_tang
        slot.vel_normal = vel_normal
        log.debug(
            "Robot %d: vel_tang=%.3f vel_normal=%.3f | ct_err=%.3f at_err=%.3f",
            rid, vel_tang, vel_normal,
            state.traj_controller.diagnostics.projection.e_ct
            if state.traj_controller.diagnostics.projection else 0.0,
            state.traj_controller.diagnostics.projection.e_at
            if state.traj_controller.diagnostics.projection else 0.0,
        )

    # ------------------------------------------------------------------
    # Path planning (unchanged)
    # ------------------------------------------------------------------

    def _replan(
        self,
        state: RobotState,
        pos: Point,
        target: Point,
        world: "World",
    ) -> None:
        rid = state.robot_id
        log.info(
            "Robot %d: replanning from (%.3f, %.3f) to (%.3f, %.3f).",
            rid, pos.x, pos.y, target.x, target.y,
        )
        try:
            path = PathPlanner(world, PATH_PLANNER_MAX_ITER).plan(pos, target)

        except Exception:
            log.warning("Robot %d: PathPlanner raised; falling back to RRT.", rid,
                        exc_info=True)
            path = []

        if not path:
            log.info("Robot %d: PathPlanner returned empty path; trying RRT.", rid)
            try:
                path = RRT(world).plan(pos, target)
            except Exception:
                log.warning("Robot %d: RRT also failed.", rid, exc_info=True)
                path = []

        if path:
            state.current_path     = path
            state.path_index       = 0
            state.last_planned_at  = time.time()
            log.info("Robot %d: new path has %d waypoints.", rid, len(path))
        else:
            log.warning("Robot %d: both planners failed; retaining old path.", rid)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        log.info("PlannerHandler running. Press Ctrl+C to stop.")
        try:
            while True:
                self.lc.handle()
        except KeyboardInterrupt:
            log.info("PlannerHandler stopped by user.")


if __name__ == "__main__":
    handler = PlannerHandler()
    handler.run()