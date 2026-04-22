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
from pathplan.main import (
    World,
    PathPlanner
)
from pathplan.rrt import RRT
from pathplan.new_bboptimizer import (
    State2D,
    PhaseState,
    AccelLimits,
    Steer2D,
    BangBangOptimizer,

)

# ---------------------------------------------------------------------------
# Motion constants
# ---------------------------------------------------------------------------
VMAX = 3.0
UMAX = Vector(0.5, 0.5)
UMIN = Vector(-0.5, -0.5)
ROBOT_RADIUS = 0.12        # metres
FIELD_MARGIN = 0.1         # metres inset from field boundary

# Tunable thresholds
TARGET_REACHED_DIST = 0.05   # metres — stop if closer than this to target
WAYPOINT_ADVANCE_DIST = 0.10  # metres — advance path_index when this close
REPLAN_TIMEOUT = 0.5          # seconds — force replan after this long
REPLAN_DEVIATION = 0.15       # metres — replan if deviated from expected waypoint
PATH_PLANNER_MAX_ITER = 1000

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
    vel_tang = vel_x * cos_a + vel_y * sin_a
    vel_normal = -vel_x * sin_a + vel_y * cos_a
    return vel_tang, vel_normal


def _build_field_boundary(field_msg, margin: float) -> Quadrilateral:
    """
    Build a Quadrilateral representing the inset field boundary.
    vision.field carries field_length and field_width in millimetres.
    """
    half_l = _mm_to_m(field_msg.field_length) / 2.0 - margin
    half_w = _mm_to_m(field_msg.field_width) / 2.0 - margin
    return Quadrilateral([
        Point(-half_l, -half_w),
        Point( half_l, -half_w),
        Point( half_l,  half_w),
        Point(-half_l,  half_w),
    ])


# ---------------------------------------------------------------------------
# Main handler
# ---------------------------------------------------------------------------

class PlannerHandler:
    """Integrates vision/GC/target LCM messages and publishes ia commands."""

    def __init__(self, team_blue: bool = False):
        self.team_blue = team_blue

        # LCM setup
        self.lc = lcm.LCM()
        self.lc.subscribe("vision", self._handle_vision)
        self.lc.subscribe("GC", self._handle_gc)
        self.lc.subscribe("TARGETS", self._handle_targets)

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
            x = float(payload["x"])
            y = float(payload["y"])
            if rid not in self.robot_states:
                log.warning("Received target for unknown robot_id=%d; ignoring.", rid)
                return
            self.robot_states[rid].target_pos = Point(x, y)
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
                # velocity is estimated by Steer2D; we keep whatever was last set
            else:
                self.robot_states[r.robot_id] = RobotState(
                    robot_id=r.robot_id,
                    current_pos=pos_m,
                    current_vel=Vector(0.0, 0.0),
                )
            log.debug("Robot %d pos=(%.3f, %.3f)", r.robot_id, pos_m.x, pos_m.y)

        # 4. Build opposing-team obstacles (constant for this tick)
        opp_obstacles = [
            Circle(
                center=Point(_mm_to_m(r.position_x), _mm_to_m(r.position_y)),
                radius=ROBOT_RADIUS,
            )
            for r in opp_robots
        ]

        # 5. Build ia output message
        msg_ia = ia()
        msg_ia.timestamp = msg.timestamp
        ia_slots: dict[int, robot] = {}

        # 6. Plan & control for each own robot
        for r_vis in own_robots:
            rid = r_vis.robot_id
            state = self.robot_states[rid]
            orientation = r_vis.orientation  # radians, world frame

            # Prepare a zero-velocity ia slot (safe default)
            slot = robot()
            slot.id = rid
            slot.spinner = 0
            slot.kick = 0
            slot.vel_tang = 0.0
            slot.vel_normal = 0.0
            slot.vel_ang = 0.0
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
        r_vis,                    # vision robot struct for this robot
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

        # b. Close enough to target → clear target, zero velocity
        if pos.distance_to(target) < TARGET_REACHED_DIST:
            log.info("Robot %d reached target (%.3f, %.3f).", rid, target.x, target.y)
            state.target_pos = None
            state.current_path = []
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
            log.debug("Robot %d: no path, replanning.", rid)
        elif now - state.last_planned_at > REPLAN_TIMEOUT:
            need_replan = True
            log.debug("Robot %d: path stale (%.1fs), replanning.", rid, now - state.last_planned_at)
        else:
            # Check deviation from expected waypoint
            expected_wp = state.current_path[min(state.path_index, len(state.current_path) - 1)]
            if pos.distance_to(expected_wp) > REPLAN_DEVIATION:
                need_replan = True
                log.debug(
                    "Robot %d: deviated %.3fm from waypoint, replanning.",
                    rid,
                    pos.distance_to(expected_wp),
                )

        if need_replan:
            self._replan(state, pos, target, world)

        if not state.current_path:
            log.warning("Robot %d: path empty after replan; zero velocity.", rid)
            return

        # d. Advance waypoint index if close enough
        while state.path_index < len(state.current_path) - 1:
            wp = state.current_path[state.path_index]
            if pos.distance_to(wp) < WAYPOINT_ADVANCE_DIST:
                state.path_index += 1
                log.debug("Robot %d: advanced to waypoint %d.", rid, state.path_index)
            else:
                break

        next_wp = state.current_path[state.path_index]

        # Control step: Steer2D + BangBangOptimizer
        vi = state.current_vel
        limits = AccelLimits(UMIN, UMAX, vmax=VMAX)
        steer = Steer2D(limits)

        path_segment = [pos, next_wp]
        x0 = State2D(
            PhaseState(pos.x, vi.x),
            PhaseState(pos.y, vi.y),
        )

        seq = steer.steer_list(path_segment, vi)

        def _collision_checker(pt: Point) -> bool:
            """Return True if point is collision-free."""
            for obs in obstacles:
                if isinstance(obs, Circle):
                    if obs.center.distance_to(pt) < obs.radius:
                        return False
            return True

        opt = BangBangOptimizer(steer, _collision_checker, world)
        result = opt.optimize(x0, seq)
        next_states = result.integrate_list(x0)

        if not next_states:
            log.warning("Robot %d: optimizer returned no states; zero velocity.", rid)
            return

        next_state: State2D = next_states[0]
        vel_x = next_state.x.v
        vel_y = next_state.y.v

        # Update velocity estimate for next tick
        state.current_vel = Vector(vel_x, vel_y)

        # Convert world-frame velocity to robot frame
        vel_tang, vel_normal = _world_vel_to_robot_frame(vel_x, vel_y, r_vis.orientation)

        slot.vel_tang = vel_tang
        slot.vel_normal = vel_normal
        log.debug(
            "Robot %d: vel_tang=%.3f vel_normal=%.3f", rid, vel_tang, vel_normal
        )

    def _replan(
        self,
        state: RobotState,
        pos: Point,
        target: Point,
        world: World,
    ) -> None:
        rid = state.robot_id
        log.info(
            "Robot %d: replanning from (%.3f, %.3f) to (%.3f, %.3f).",
            rid, pos.x, pos.y, target.x, target.y,
        )
        try:
            path = PathPlanner(world, PATH_PLANNER_MAX_ITER).plan(pos, target)
        except Exception:
            log.warning("Robot %d: PathPlanner raised; falling back to RRT.", rid, exc_info=True)
            path = []

        if not path:
            log.info("Robot %d: PathPlanner returned empty path; trying RRT.", rid)
            try:
                path = RRT(world).plan(pos, target)
            except Exception:
                log.warning("Robot %d: RRT also failed.", rid, exc_info=True)
                path = []

        if path:
            state.current_path = path
            state.path_index = 0
            state.last_planned_at = time.time()
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