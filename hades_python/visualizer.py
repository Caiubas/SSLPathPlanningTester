"""
visualizer.py — Real-time PyGame debug visualizer for the SSL robot soccer system.

Reads LCM channels:
  - "vision" → data.vision   (robot positions, ball, field geometry — mm)

Publishes:
  - "IA"      → data.ia      (robot velocity commands, computed internally)
  - "TARGETS" → JSON bytes {"robot_id": int, "x": float, "y": float}  (metres)

Architecture
------------
- Main thread    : PyGame event loop + rendering (Renderer)
- LCMThread      : reads "vision", writes SharedState under a Lock
- PlannerThread  : runs PlannerHandler tick each vision frame; writes paths and
                   ia_commands directly into SharedState (no LCM roundtrip for IA)
- SharedState    : single source of truth; snapshotted each frame before drawing

Target workflow
---------------
1. Left-click a robot of the active team  → selects it.
2. Left-click anywhere on the field       → sets that robot's target (metres).
3. PlannerThread detects the new target, plans a path, and publishes velocity
   commands to SharedState.ia_commands every vision tick until target reached.
4. Press C to clear the selected robot's target.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Optional
from control import TrajectoryController, TrajectoryPoint

import pygame

# ---------------------------------------------------------------------------
# Optional PlannerHandler import
# ---------------------------------------------------------------------------
try:
    from planner_handler import PlannerHandler as _PlannerHandler  # type: ignore
    _PLANNER_AVAILABLE = True
except ImportError:
    _PLANNER_AVAILABLE = False
    print("[WARN] planner_handler not found – velocity commands will not be computed.")

# ---------------------------------------------------------------------------
# Optional LCM import – visualizer degrades gracefully without it so the
# rendering logic can be tested in isolation.
# ---------------------------------------------------------------------------
try:
    import lcm  # type: ignore
    _LCM_AVAILABLE = True
except ImportError:
    _LCM_AVAILABLE = False
    print("[WARN] lcm not found – running in offline / demo mode.")

# ---------------------------------------------------------------------------
# Optional data package imports (generated LCM types)
# ---------------------------------------------------------------------------
try:
    from data import vision as vision_t  # type: ignore
    _DATA_AVAILABLE = True
except ImportError:
    _DATA_AVAILABLE = False
    print("[WARN] data package not found – using stub types.")

# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------
try:
    from pathplan.main import Point, Vector  # type: ignore
except ImportError:
    @dataclass
    class Point:  # type: ignore[no-redef]
        """Minimal Point stub used when geometry package is absent."""
        x: float = 0.0
        y: float = 0.0

    @dataclass
    class Vector:  # type: ignore[no-redef]
        """Minimal Vector stub used when geometry package is absent."""
        x: float = 0.0
        y: float = 0.0


# ---------------------------------------------------------------------------
# SSL Division B field defaults (mm)
# ---------------------------------------------------------------------------
_DIV_B = {
    "field_length": 9_000.0,
    "field_width":  6_000.0,
    "boundary_width": 300.0,
    "goal_width":   1_000.0,
    "goal_depth":     200.0,
    "penalty_area_depth":  1_000.0,
    "penalty_area_width":  2_000.0,
    "center_circle_radius": 500.0,
}

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------
C = {
    "field":       (  34, 139,  34),
    "line":        ( 255, 255, 255),
    "penalty":     ( 220, 220, 220),
    "yellow_bot":  ( 255, 215,   0),
    "blue_bot":    (  30, 144, 255),
    "ball":        ( 255, 140,   0),
    "select":      ( 255,  50,  50),
    "waypoint":    ( 255, 255,   0),
    "vel_arrow":   (   0, 255, 128),
    "hud_bg":      (   0,   0,   0, 180),
    "hud_text":    ( 230, 230, 230),
    "white":       ( 255, 255, 255),
    "dark":        (  20,  20,  20),
    "orient_arrow": (255, 255, 255),
}

# ---------------------------------------------------------------------------
# Shared-state data structures
# ---------------------------------------------------------------------------

@dataclass
class RobotState:
    """Snapshot of a single robot's state (positions in mm, angle in rad)."""
    robot_id: int = 0
    x: float = 0.0          # mm
    y: float = 0.0          # mm
    orientation: float = 0.0  # rad
    team: str = "yellow"     # "yellow" | "blue"
    vel_tang: float = 0.0
    vel_normal: float = 0.0


@dataclass
class BallState:
    """Snapshot of the ball (mm)."""
    x: float = 0.0
    y: float = 0.0


@dataclass
class FieldGeometry:
    """Field dimensions in mm."""
    field_length: float = _DIV_B["field_length"]
    field_width:  float = _DIV_B["field_width"]
    boundary_width: float = _DIV_B["boundary_width"]
    goal_width:   float = _DIV_B["goal_width"]
    goal_depth:   float = _DIV_B["goal_depth"]
    penalty_area_depth: float = _DIV_B["penalty_area_depth"]
    penalty_area_width: float = _DIV_B["penalty_area_width"]
    center_circle_radius: float = _DIV_B["center_circle_radius"]


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    robots: dict[int, RobotState] = field(default_factory=dict)
    ball: BallState = field(default_factory=BallState)
    geometry: FieldGeometry = field(default_factory=FieldGeometry)
    timestamp: float = 0.0
    team_blue: bool = False          # ← ADICIONAR
    paths: dict[int, list[Point]] = field(default_factory=dict)
    ia_commands: dict[int, tuple[float, float]] = field(default_factory=dict)
    targets: dict[int, "Point"] = field(default_factory=dict)
    paused: bool = False
    vision_event: threading.Event = field(default_factory=threading.Event)

# ---------------------------------------------------------------------------
# LCM thread
# ---------------------------------------------------------------------------

class LCMThread(threading.Thread):
    """
    Background daemon thread that runs the LCM handle loop.

    Writes decoded vision and IA messages into *shared* under its lock.
    """

    def __init__(self, shared: SharedState) -> None:
        super().__init__(name="LCMThread", daemon=True)
        self._shared = shared
        self._stop_event = threading.Event()
        self._lc: Optional[object] = None  # lcm.LCM instance

    # ------------------------------------------------------------------
    def stop(self) -> None:
        """Signal the thread to stop and unblock the handle loop."""
        self._stop_event.set()

    # ------------------------------------------------------------------
    def run(self) -> None:  # noqa: C901
        """Main LCM receive loop."""
        if not _LCM_AVAILABLE:
            print("[LCMThread] LCM unavailable – thread idle.")
            return

        import lcm as _lcm  # local import so module loads without lcm

        self._lc = _lcm.LCM()
        self._lc.subscribe("vision", self._vision_handler)
        #self._lc.subscribe("GC", self._gc_handler)

        print("[LCMThread] subscribed to 'vision'.")

        fd = self._lc.fileno()
        import select as _select

        while not self._stop_event.is_set():
            # Non-blocking poll so we can check stop_event regularly.
            ready, _, _ = _select.select([fd], [], [], 0.05)
            if ready:
                try:
                    self._lc.handle()
                except Exception as exc:
                    print(f"[LCMThread] handle error: {exc}")

        print("[LCMThread] exiting.")

    # ------------------------------------------------------------------
    def _vision_handler(self, channel: str, raw: bytes) -> None:  # noqa: ARG002
        """Decode a vision LCM message and update SharedState."""
        if self._shared.paused:
            return
        if not _DATA_AVAILABLE:
            return
        try:
            msg = vision_t.decode(raw)
        except Exception as exc:
            print(f"[LCMThread] vision decode error: {exc}")
            return

        robots: dict[int, RobotState] = {}

        # Fixed arrays of 16; robots_yellow_size / robots_blue_size are the
        # logical counts of live robots — slice to ignore uninitialised slots.
        for r in msg.robots_yellow[: msg.robots_yellow_size]:
            if not r.detected:
                continue
            robots[r.robot_id] = RobotState(
                robot_id=r.robot_id,
                x=r.position_x,
                y=r.position_y,
                orientation=r.orientation,
                team="yellow",
            )
        for r in msg.robots_blue[: msg.robots_blue_size]:
            if not r.detected:
                continue
            # Use negative IDs for blue to avoid key collision with yellow.
            key = -(r.robot_id + 1)
            robots[key] = RobotState(
                robot_id=r.robot_id,
                x=r.position_x,
                y=r.position_y,
                orientation=r.orientation,
                team="blue",
            )

        # balls is a single detection_balls struct (not a list)
        ball = BallState(
            x=msg.balls.position_x,
            y=msg.balls.position_y,
        )

        # Geometry lives in msg.field; map .lcm field names → FieldGeometry attrs.
        g = msg.field
        geom = FieldGeometry(
            field_length          = float(g.field_length),
            field_width           = float(g.field_width),
            boundary_width        = float(g.boundary_width),
            goal_width            = float(g.goal_width),
            goal_depth            = float(g.goal_depth),
            penalty_area_depth    = float(g.defense_area_height),
            penalty_area_width    = float(g.defense_area_width),
            center_circle_radius  = float(g.center_circle_radius),
        )

        with self._shared.lock:
            self._shared.robots    = robots
            self._shared.ball      = ball
            self._shared.geometry  = geom
            self._shared.timestamp = float(msg.timestamp)

        # Wake PlannerThread for this vision frame.
        self._shared.vision_event.set()

    def _gc_handler(self, channel: str, raw: bytes) -> None:
        try:
            from data.game_controller import game_controller as gc_t
            msg = gc_t.decode(raw)
            with self._shared.lock:
                self._shared.team_blue = bool(msg.team_blue)
        except Exception as exc:
            print(f"[LCMThread] GC decode error: {exc}")


# ---------------------------------------------------------------------------
# Planner thread
# ---------------------------------------------------------------------------

class PlannerThread(threading.Thread):
    """
    Background daemon thread that runs the motion planner once per vision frame.

    On each wake-up it:
      1. Snapshots robots, geometry, and targets from SharedState.
      2. Syncs those into an internal _PlannerHandler (robot states, field).
      3. Calls _plan_and_control for every own robot that has a pending target.
      4. Writes computed (vel_tang, vel_normal) back into SharedState.ia_commands.
      5. Writes planned waypoint paths into SharedState.paths (metres → mm for display).
      6. Publishes an IA LCM message if LCM is available.

    If planner_handler or its dependencies are not importable the thread does
    nothing (graceful degradation).
    """

    def __init__(
        self,
        shared: SharedState,
        team_blue: bool = False,
        lc_pub=None,
    ) -> None:
        super().__init__(name="PlannerThread", daemon=True)
        self._shared    = shared
        self._team_blue = team_blue
        self._lc_pub    = lc_pub
        self._stop_event = threading.Event()

        # Internal planner — created lazily once dependencies confirm available.
        self._handler: Optional[object] = None

    # ------------------------------------------------------------------
    def stop(self) -> None:
        self._stop_event.set()
        self._shared.vision_event.set()   # unblock wait

    # ------------------------------------------------------------------
    def run(self) -> None:
        if not _PLANNER_AVAILABLE:
            print("[PlannerThread] planner_handler unavailable – thread idle.")
            return

        # Lazy import of planner internals (avoids import-time failures)
        try:
            from planner_handler import (  # type: ignore
                ROBOT_RADIUS, VMAX, UMAX, UMIN,
                TARGET_REACHED_DIST,
                REPLAN_TIMEOUT, REPLAN_DEVIATION,
                PATH_PLANNER_MAX_ITER, FIELD_MARGIN,
                _mm_to_m, _world_vel_to_robot_frame,
                _build_trajectory,
                RobotState as PlannerRobotState,
            )
            from pathplan.main import Point as Pt, Vector as Vec, Circle, Quadrilateral  # type: ignore
            from pathplan.main import World  # type: ignore
            from pathplan.rrt import RRT  # type: ignore
            from pathplan.c_path import PathPlanner
        except ImportError as exc:
            print(f"[PlannerThread] import error: {exc} - thread idle.")
            return

        # Resolve the LCM ia / robot classes once at thread startup.
        import traceback as _tb
        _ia_cls    = None
        _robot_cls = None
        if _LCM_AVAILABLE and _DATA_AVAILABLE:
            # Strategy 1 – class re-exported from data/__init__.py
            try:
                import data as _data_pkg  # type: ignore
                _ia_sym    = getattr(_data_pkg, 'ia',    None)
                _robot_sym = getattr(_data_pkg, 'robot', None)
                if callable(_ia_sym) and callable(_robot_sym):
                    _ia_cls, _robot_cls = _ia_sym, _robot_sym
                    print('[PlannerThread] LCM classes via data.__init__')
            except Exception:
                pass
            # Strategy 2 – class inside sub-module data/ia.py
            if _ia_cls is None:
                try:
                    import importlib as _il
                    _ia_mod    = _il.import_module('data.ia')
                    _robot_mod = _il.import_module('data.robot')
                    _ia_sym    = getattr(_ia_mod,    'ia',    None)
                    _robot_sym = getattr(_robot_mod, 'robot', None)
                    if callable(_ia_sym) and callable(_robot_sym):
                        _ia_cls, _robot_cls = _ia_sym, _robot_sym
                        print('[PlannerThread] LCM classes via data.ia sub-module')
                    else:
                        print(f'[PlannerThread] WARNING: ia={_ia_sym!r} robot={_robot_sym!r} not callable')
                except Exception:
                    print('[PlannerThread] WARNING: could not resolve LCM classes:')
                    _tb.print_exc()

        # Internal planner state: robot_id → PlannerRobotState
        # PlannerRobotState now carries a traj_controller field (TrajectoryController).
        planner_states: dict[int, PlannerRobotState] = {}
        field_boundary: Optional[Quadrilateral] = None

        # Track the last known target per robot so we can detect target changes
        # and reset the trajectory controller only when necessary.
        last_targets: dict[int, Pt] = {}

        import time as _time

        print("[PlannerThread] running.")

        while not self._stop_event.is_set():
            # Block until a new vision frame arrives (or stop is signalled).
            self._shared.vision_event.wait(timeout=0.1)
            self._shared.vision_event.clear()

            if self._stop_event.is_set():
                break

            # ── Snapshot shared state ────────────────────────────────────
            with self._shared.lock:
                if self._shared.paused:
                    continue
                vis_robots  = dict(self._shared.robots)   # key → RobotState(vis)
                targets_in  = dict(self._shared.targets)  # robot_id → Point(m)
                geom        = self._shared.geometry
                team_blue   = self._shared.team_blue

            # ── Rebuild field boundary ───────────────────────────────────
            half_l = geom.field_length / 2000.0 - FIELD_MARGIN   # mm→m
            half_w = geom.field_width  / 2000.0 - FIELD_MARGIN
            field_boundary = Quadrilateral([
                Pt(-half_l, -half_w),
                Pt( half_l, -half_w),
                Pt( half_l,  half_w),
                Pt(-half_l,  half_w),
            ])

            # ── Split robots by team ─────────────────────────────────────
            own_vis = {k: r for k, r in vis_robots.items()
                       if (r.team == "blue") == team_blue}
            opp_vis = {k: r for k, r in vis_robots.items()
                       if (r.team == "blue") != team_blue}

            opp_obstacles = [
                Circle(
                    center=Pt(r.x / 1000.0, r.y / 1000.0),
                    radius=ROBOT_RADIUS,
                )
                for r in opp_vis.values()
            ]

            # ── Sync planner robot states ────────────────────────────────
            for vis_r in own_vis.values():
                rid   = vis_r.robot_id
                pos_m = Pt(vis_r.x / 1000.0, vis_r.y / 1000.0)
                if rid not in planner_states:
                    planner_states[rid] = PlannerRobotState(
                        robot_id=rid,
                        current_pos=pos_m,
                        current_vel=Vec(0.0, 0.0),
                    )
                else:
                    planner_states[rid].current_pos = pos_m

            # Propagate targets; reset trajectory controller when target changes.
            for rid, tgt in targets_in.items():
                if rid not in planner_states:
                    continue
                pstate = planner_states[rid]
                prev = last_targets.get(rid)
                if prev is None or abs(prev.x - tgt.x) > 1e-4 or abs(prev.y - tgt.y) > 1e-4:
                    # New or changed target — clear path so replan triggers below,
                    # and reset the PID integrators to avoid transient spikes.
                    pstate.target_pos   = tgt
                    pstate.current_path = []
                    pstate.traj_controller.reset()
                    last_targets[rid]   = tgt
                else:
                    pstate.target_pos = tgt

            # ── Plan & control ───────────────────────────────────────────
            new_ia:    dict[int, tuple[float, float]] = {}
            new_paths: dict[int, list] = {}  # vis_key → list of {"pos", "vel"}

            own_vis_list = list(own_vis.values())

            for vis_r in own_vis_list:
                rid = vis_r.robot_id
                if rid not in planner_states:
                    continue
                pstate = planner_states[rid]
                if pstate.target_pos is None:
                    continue

                pos    = pstate.current_pos
                target = pstate.target_pos

                # ── Target-reached check ─────────────────────────────────
                if pos.distance_to(target) < TARGET_REACHED_DIST and pstate.current_vel.x == 0.0 and pstate.current_vel.y == 0.0 and False:
                    print(f"[PlannerThread] Robot {rid} reached target.")
                    pstate.target_pos   = None
                    pstate.current_path = []
                    pstate.current_vel  = Vec(0.0, 0.0)
                    pstate.traj_controller.reset()
                    last_targets.pop(rid, None)
                    with self._shared.lock:
                        self._shared.targets.pop(rid, None)
                    continue

                world = World(
                    obstacles=opp_obstacles + [
                        Circle(
                            center=Pt(r.x / 1000.0, r.y / 1000.0),
                            radius=ROBOT_RADIUS,
                        )
                        for r in own_vis_list
                        if r.robot_id != rid
                    ],
                    boundaries=field_boundary,
                )

                # ── Replan logic ─────────────────────────────────────────
                now = _time.time()
                need_replan = (
                    not pstate.current_path
                    or now - pstate.last_planned_at > REPLAN_TIMEOUT
                )
                if need_replan: print("REPLAN TIMEOUT", now - pstate.last_planned_at > REPLAN_TIMEOUT)
                if not need_replan and pstate.current_path:
                    expected = pstate.current_path[
                        min(pstate.path_index, len(pstate.current_path) - 1)
                    ]

                if need_replan:
                    path: list = []
                    try:
                        path = PathPlanner(world, PATH_PLANNER_MAX_ITER).plan(pos, target)
                        if path: print("planner planned")
                    except Exception:
                        pass
                    if not path:
                        try:
                            path = RRT(world).plan(pos, target)
                            if path: print("RRT planned")
                        except Exception:
                            pass
                    if path:
                        pstate.current_path    = path
                        pstate.path_index      = 0
                        pstate.last_planned_at = _time.time()
                        # Build TrajectoryPoint list with reference velocities from
                        # BangBangOptimizer (falls back to cruise profile on error).
                        traj = _build_trajectory(path, pstate.current_vel, world)
                        pstate.traj_controller.set_trajectory(traj)

                if not pstate.current_path:
                    continue

                # ── PID trajectory-following control ─────────────────────
                # TrajectoryController handles:
                #   • orthogonal projection onto the current segment
                #   • separate PID for cross-track and along-track errors
                #   • velocity feedforward from the reference trajectory
                #   • waypoint advancement (distance + projection-parameter check)
                try:
                    cmd: Vec = pstate.traj_controller.update(pos, now=_time.monotonic())

                    # Sync legacy path_index with the controller's waypoint index
                    # so the deviation-based replan check above stays meaningful.
                    pstate.path_index = min(
                        pstate.traj_controller.waypoint_index,
                        len(pstate.current_path) - 1,
                    )

                    # Carry the commanded velocity forward as the estimated state
                    # for the next BangBang trajectory build.
                    pstate.current_vel = cmd

                    vt, vn = _world_vel_to_robot_frame(
                        cmd.x, cmd.y, vis_r.orientation
                    )
                    new_ia[rid] = (vt, vn)

                except Exception as exc:
                    print(f"[PlannerThread] control error robot {rid}: {exc}")

                # ── Path data for the visualizer (mm) ────────────────────
                vis_key = next(
                    (k for k, r in vis_robots.items() if r.robot_id == rid), None
                )
                if vis_key is not None:
                    # Expose TrajectoryController waypoints with their reference
                    # velocities so the renderer can draw the planned trajectory.
                    traj_pts = pstate.traj_controller._trajectory  # list[TrajectoryPoint]
                    new_paths[vis_key] = [
                        {
                            "pos": (tp.position.x * 1000.0, tp.position.y * 1000.0),  # mm
                            "vel": (tp.velocity.x, tp.velocity.y),                     # m/s
                        }
                        for tp in traj_pts
                    ]

            # ── Write results back to SharedState ────────────────────────
            with self._shared.lock:
                self._shared.ia_commands.update(new_ia)
                # Remove ia_commands for robots that no longer have active targets
                active_rids = {
                    r.robot_id for r in own_vis.values()
                    if planner_states.get(r.robot_id) and
                       planner_states[r.robot_id].target_pos is not None
                }
                for k in list(self._shared.ia_commands.keys()):
                    if k not in active_rids:
                        del self._shared.ia_commands[k]
                self._shared.paths.update(new_paths)

            # ── Publish IA over LCM if available ─────────────────────────
            if self._lc_pub is not None and _LCM_AVAILABLE and _DATA_AVAILABLE and _ia_cls is not None:
                try:
                    msg_ia             = _ia_cls()
                    msg_ia.timestamp   = 0
                    msg_ia.estrategia  = 0
                    msg_ia.processo    = 0
                    msg_ia.robots_size = len(new_ia)

                    # ia.robots is a fixed array of exactly 16 slots.
                    slots = [_robot_cls() for _ in range(16)]
                    for idx, (rid, (vt, vn)) in enumerate(new_ia.items()):
                        if idx >= 16:
                            break
                        s              = slots[idx]
                        s.id           = rid
                        s.vel_tang     = vt
                        s.vel_normal   = vn
                        s.vel_ang      = 0.0
                        s.kick         = 0
                        s.spinner      = 0
                        s.kick_speed_x = 0.0
                        s.kick_speed_z = 0.0

                    msg_ia.robots = slots
                    self._lc_pub.publish("IA", msg_ia.encode())
                except Exception as exc:
                    print(f"[PlannerThread] IA publish error: {exc}")
                    _tb.print_exc()

        print("[PlannerThread] exiting.")



# ---------------------------------------------------------------------------
# Coordinate transform helper
# ---------------------------------------------------------------------------

class CoordTransform:
    """
    Converts between field coordinates (mm, origin at centre) and screen
    pixels.  Recomputed whenever the window is resized.
    """

    def __init__(
        self,
        win_w: int,
        win_h: int,
        geom: FieldGeometry,
    ) -> None:
        self.win_w = win_w
        self.win_h = win_h
        self.geom  = geom
        self._compute()

    # ------------------------------------------------------------------
    def _compute(self) -> None:
        """Derive scale and offset so the field fills the window with margin."""
        g = self.geom
        # Total extent including boundary
        total_w = g.field_length + 2 * g.boundary_width
        total_h = g.field_width  + 2 * g.boundary_width
        # Scale: fit inside window preserving aspect ratio
        sx = self.win_w / total_w
        sy = self.win_h / total_h
        self.scale = min(sx, sy) * 0.95   # 5 % padding
        # Pixel size of the playing field (no boundary)
        self.field_px_w = g.field_length * self.scale
        self.field_px_h = g.field_width  * self.scale
        # Top-left corner of the playing field in screen coords
        self.field_x0 = (self.win_w - self.field_px_w) / 2
        self.field_y0 = (self.win_h - self.field_px_h) / 2

    # ------------------------------------------------------------------
    def mm_to_px(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """
        Convert field-centre mm coordinates to screen pixels.

        The SSL field has its origin at the centre; +x points right,
        +y points up.  Screen coords have +y pointing down.
        """
        px = self.field_x0 + (x_mm + self.geom.field_length / 2) * self.scale
        py = self.field_y0 + (self.geom.field_width  / 2 - y_mm) * self.scale
        return int(px), int(py)

    # ------------------------------------------------------------------
    def px_to_mm(self, px: int, py: int) -> tuple[float, float]:
        """Inverse of mm_to_px — returns field-centre mm coordinates."""
        x_mm = (px - self.field_x0) / self.scale - self.geom.field_length / 2
        y_mm = self.geom.field_width  / 2 - (py - self.field_y0) / self.scale
        return x_mm, y_mm

    # ------------------------------------------------------------------
    def scale_mm(self, mm: float) -> int:
        """Scale a length from mm to pixels (no offset)."""
        return max(1, int(mm * self.scale))

    # ------------------------------------------------------------------
    def robot_radius_px(self) -> int:
        """Standard SSL robot radius (90 mm) in pixels."""
        return self.scale_mm(90)

    # ------------------------------------------------------------------
    def update(self, win_w: int, win_h: int, geom: FieldGeometry) -> None:
        """Recompute after a window resize or geometry change."""
        self.win_w = win_w
        self.win_h = win_h
        self.geom  = geom
        self._compute()


# ---------------------------------------------------------------------------
# Renderer
# ---------------------------------------------------------------------------

class Renderer:
    """
    Owns all PyGame draw calls.  Receives a snapshot of SharedState each
    frame so it never holds the lock while drawing.
    """

    _DASH_LEN = 8    # pixels per dash segment
    _DASH_GAP = 6    # pixels per gap segment
    _BOT_R_MM = 90   # robot radius in mm

    def __init__(self, screen: pygame.Surface) -> None:
        self._screen = screen
        pygame.font.init()
        self._font_sm = pygame.font.SysFont("monospace", 13)
        self._font_md = pygame.font.SysFont("monospace", 15, bold=True)
        self._font_lg = pygame.font.SysFont("monospace", 18, bold=True)

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def draw_frame(
        self,
        robots:       dict[int, RobotState],
        ball:         BallState,
        geom:         FieldGeometry,
        paths:        dict[int, list[Point]],
        ia_commands:  dict[int, tuple[float, float]],
        targets:      dict[int, "Point"],
        selected_key: Optional[int],
        active_team:  str,
        timestamp:    float,
        paused:       bool,
        fps:          float,
    ) -> None:
        """
        Draw one complete frame.

        Parameters
        ----------
        robots:       robot key → RobotState (yellow keys ≥ 0, blue keys < 0)
        ball:         current ball state
        geom:         field geometry
        paths:        robot key → list[Point] waypoints (mm)
        ia_commands:  robot_id (real id) → (vel_tang, vel_normal)
        targets:      robot_id (real id) → target Point (metres)
        selected_key: key of the selected robot (or None)
        active_team:  "yellow" | "blue"
        timestamp:    LCM timestamp
        paused:       whether LCM processing is paused
        fps:          current rendering fps
        """
        w, h = self._screen.get_size()
        tf = CoordTransform(w, h, geom)

        self._screen.fill(C["dark"])
        self._draw_field(tf, geom)
        self._draw_targets(tf, robots, targets)
        self._draw_robots(tf, robots, paths, ia_commands, selected_key, active_team)
        self._draw_ball(tf, ball)
        self._draw_hud(
            robots, ball, selected_key, active_team, timestamp, paused, fps,
            ia_commands, targets,
        )


    # ------------------------------------------------------------------
    # Target markers
    # ------------------------------------------------------------------

    def _draw_targets(
        self,
        tf:      CoordTransform,
        robots:  dict[int, RobotState],
        targets: dict[int, "Point"],
    ) -> None:
        """Draw a crosshair at each robot's active target (metres → pixels)."""
        # Build robot_id → team colour map
        id_colour: dict[int, tuple] = {}
        for r in robots.values():
            id_colour[r.robot_id] = (
                C["yellow_bot"] if r.team == "yellow" else C["blue_bot"]
            )

        for rid, tgt in targets.items():
            col = id_colour.get(rid, C["white"])
            cx, cy = tf.mm_to_px(tgt.x * 1000.0, tgt.y * 1000.0)
            arm = max(6, tf.robot_radius_px() // 2)
            # Crosshair lines
            pygame.draw.line(self._screen, col, (cx - arm, cy), (cx + arm, cy), 2)
            pygame.draw.line(self._screen, col, (cx, cy - arm), (cx, cy + arm), 2)
            # Small circle
            pygame.draw.circle(self._screen, col, (cx, cy), arm, 1)

    # ------------------------------------------------------------------
    # Field
    # ------------------------------------------------------------------

    def _draw_field(self, tf: CoordTransform, g: FieldGeometry) -> None:
        """Draw the green pitch and all white field markings."""
        # Green rectangle
        x0, y0 = tf.mm_to_px(-g.field_length / 2, g.field_width / 2)
        x1, y1 = tf.mm_to_px( g.field_length / 2, -g.field_width / 2)
        rect = pygame.Rect(x0, y0, x1 - x0, y1 - y0)
        pygame.draw.rect(self._screen, C["field"], rect)

        lw = max(1, tf.scale_mm(20))   # line width ≈ 20 mm

        # Boundary / touchlines
        pygame.draw.rect(self._screen, C["line"], rect, lw)

        # Halfway line
        mid_top    = tf.mm_to_px(0,  g.field_width / 2)
        mid_bottom = tf.mm_to_px(0, -g.field_width / 2)
        pygame.draw.line(self._screen, C["line"], mid_top, mid_bottom, lw)

        # Centre circle
        cx, cy = tf.mm_to_px(0, 0)
        r_px = tf.scale_mm(g.center_circle_radius)
        pygame.draw.circle(self._screen, C["line"], (cx, cy), r_px, lw)

        # Centre dot
        pygame.draw.circle(self._screen, C["line"], (cx, cy), max(2, lw))

        # Penalty areas (left and right)
        for side in (-1, 1):
            x_near = side * g.field_length / 2
            x_far  = x_near - side * g.penalty_area_depth
            y_top  =  g.penalty_area_width / 2
            y_bot  = -g.penalty_area_width / 2
            pa0 = tf.mm_to_px(x_far, y_top)
            pa1 = tf.mm_to_px(x_near, y_bot)
            pa_rect = pygame.Rect(
                min(pa0[0], pa1[0]), min(pa0[1], pa1[1]),
                abs(pa1[0] - pa0[0]), abs(pa1[1] - pa0[1]),
            )
            pygame.draw.rect(self._screen, C["line"], pa_rect, lw)

            # Goal boxes
            gx_near = x_near
            gx_far  = x_near + side * g.goal_depth
            gy_top  =  g.goal_width / 2
            gy_bot  = -g.goal_width / 2
            g0 = tf.mm_to_px(gx_far, gy_top)
            g1 = tf.mm_to_px(gx_near, gy_bot)
            goal_rect = pygame.Rect(
                min(g0[0], g1[0]), min(g0[1], g1[1]),
                abs(g1[0] - g0[0]), abs(g1[1] - g0[1]),
            )
            pygame.draw.rect(self._screen, C["line"], goal_rect, lw)

    # ------------------------------------------------------------------
    # Robots
    # ------------------------------------------------------------------

    def _draw_robots(
            self,
            tf: CoordTransform,
            robots: dict[int, RobotState],
            paths: dict[int, list[Point]],
            ia_commands: dict[int, tuple[float, float]],
            selected_key: Optional[int],
            active_team: str,
    ) -> None:
        """Draw all robots, highlights, paths, and velocity arrows."""
        r_px = tf.robot_radius_px()

        for key, robot in robots.items():
            cx, cy = tf.mm_to_px(robot.x, robot.y)
            colour = C["yellow_bot"] if robot.team == "yellow" else C["blue_bot"]

            # Body
            pygame.draw.circle(self._screen, colour, (cx, cy), r_px)

            # Active-team thicker outline
            team_match = robot.team == active_team
            outline_w = max(1, r_px // 5) if team_match else 1
            outline_c = C["white"] if team_match else (180, 180, 180)
            pygame.draw.circle(self._screen, outline_c, (cx, cy), r_px, outline_w)

            # Selection highlight
            if key == selected_key:
                pygame.draw.circle(
                    self._screen, C["select"], (cx, cy), r_px + 4, 3
                )

            # Orientation arrow (frente do robô)
            angle = robot.orientation
            arr_len = r_px * 1.4
            ax = cx + int(arr_len * math.cos(angle))
            ay = cy - int(arr_len * math.sin(angle))
            pygame.draw.line(self._screen, C["orient_arrow"], (cx, cy), (ax, ay), max(1, r_px // 6))

            # Robot ID
            label = self._font_sm.render(str(robot.robot_id), True, C["white"])
            lx = cx - label.get_width() // 2
            ly = cy - label.get_height() // 2
            self._screen.blit(label, (lx, ly))

            # Waypoint path (selected robot only)
            # No Renderer._draw_robots, substitua a seção do Waypoint path:

            # Waypoint path (selected robot only)
            # Waypoint path (selected robot only)
            # No Renderer._draw_robots, dentro da parte de waypoints:

            if key == selected_key and key in paths and len(paths[key]) > 1:
                path_data = paths[key]

                # 1. Desenhar a linha base (o "fio" da trajetória)
                pts_px = [tf.mm_to_px(p["pos"][0], p["pos"][1]) for p in path_data]
                self._draw_dashed_polyline(pts_px, (100, 100, 100), width=1)

                # 2. Iterar sobre os dados para desenhar PONTOS e VELOCIDADES
                for data in path_data[::3]:  # [::3] para não sobrepor tudo, ajuste conforme necessário
                    px, py = tf.mm_to_px(data["pos"][0], data["pos"][1])
                    vx, vy = data["vel"]

                    # --- DESENHO DO PONTO (Estado) ---
                    # Mantendo os pontos que marcam a troca de estado/posição
                    pygame.draw.circle(self._screen, C["waypoint"], (px, py), 3)

                    # --- DESENHO DA VELOCIDADE (Vetor) ---
                    speed = math.hypot(vx, vy)
                    if speed > 0.05:
                        v_scale = 40
                        ex = px + int(vx * v_scale)
                        ey = py - int(vy * v_scale)

                        # Seta de velocidade saindo do ponto
                        pygame.draw.line(self._screen, C["waypoint"], (px, py), (ex, ey), 2)
                        self._draw_mini_arrowhead(self._screen, C["waypoint"], (px, py), (ex, ey))

            # --- ADIÇÃO: VETOR DE VELOCIDADE ---
            # O dicionário ia_commands usa o robot_id real (inteiro positivo)
            actual_id = robot.robot_id
            if actual_id in ia_commands:
                vt, vn = ia_commands[actual_id]
                speed = math.hypot(vt, vn)

                if speed > 0.05:  # Filtro de ruído/velocidade mínima
                    # Converter velocidade local (robot frame) para global (field frame)
                    # No SSL: x_global = vt*cos(theta) - vn*sin(theta)
                    #         y_global = vt*sin(theta) + vn*cos(theta)
                    vx_global = vt * math.cos(angle) - vn * math.sin(angle)
                    vy_global = vt * math.sin(angle) + vn * math.cos(angle)

                    # Escalonamento para visualização (ex: 1m/s = 100px de vetor)
                    # Ajuste o fator 'scale_factor' conforme a necessidade de visibilidade
                    scale_factor = tf.scale_mm(400)  # Referência visual de 400mm

                    # Fim do vetor em pixels
                    # Note o sinal de menos no Y pois o Pygame inverte o eixo Y da tela
                    ex = cx + int(vx_global * scale_factor)
                    ey = cy - int(vy_global * scale_factor)

                    # Desenha a linha do vetor e a ponta da seta
                    pygame.draw.line(self._screen, C["vel_arrow"], (cx, cy), (ex, ey), 3)
                    self._draw_arrowhead(self._screen, C["vel_arrow"], (cx, cy), (ex, ey), size=10)

    # ------------------------------------------------------------------
    # Ball
    # ------------------------------------------------------------------

    def _draw_ball(self, tf: CoordTransform, ball: BallState) -> None:
        """Draw the orange ball."""
        cx, cy = tf.mm_to_px(ball.x, ball.y)
        r_px = max(4, tf.scale_mm(43))   # SSL ball radius ≈ 43 mm
        pygame.draw.circle(self._screen, C["ball"], (cx, cy), r_px)
        pygame.draw.circle(self._screen, C["white"], (cx, cy), r_px, 1)

    # ------------------------------------------------------------------
    # HUD
    # ------------------------------------------------------------------

    def _draw_hud(
        self,
        robots:       dict[int, RobotState],
        ball:         BallState,
        selected_key: Optional[int],
        active_team:  str,
        timestamp:    float,
        paused:       bool,
        fps:          float,
        ia_commands:  dict[int, tuple[float, float]],
        targets:      dict[int, "Point"],
    ) -> None:
        """Draw the semi-transparent info overlay in the top-left corner."""
        lines: list[str] = [
            f"FPS   : {fps:5.1f}",
            f"Time  : {timestamp:.3f}",
            f"Team  : {active_team.upper()}",
            f"Ball  : ({ball.x/1000:.2f} m, {ball.y/1000:.2f} m)",
        ]

        if selected_key is not None and selected_key in robots:
            r = robots[selected_key]
            lines += [
                "─" * 24,
                f"Robot : #{r.robot_id} [{r.team}]",
                f"Pos   : ({r.x/1000:.2f},{r.y/1000:.2f}) m",
                f"Angle : {r.orientation:.3f} rad",
            ]
            # Show target and velocity commands if robot has an active target
            if r.robot_id in targets:
                tgt = targets[r.robot_id]
                lines.append(f"Target: ({tgt.x:.2f},{tgt.y:.2f}) m")
            if r.robot_id in ia_commands:
                vt, vn = ia_commands[r.robot_id]
                speed = math.hypot(vt, vn)
                lines += [
                    f"Vt    : {vt:+.3f} m/s",
                    f"Vn    : {vn:+.3f} m/s",
                    f"Speed : {speed:.3f} m/s",
                ]

        if paused:
            lines.insert(0, "⏸  PAUSED  ⏸")

        padding = 8
        line_h  = self._font_sm.get_height() + 3
        box_w   = 220
        box_h   = len(lines) * line_h + 2 * padding

        hud_surf = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
        hud_surf.fill((0, 0, 0, 170))
        self._screen.blit(hud_surf, (10, 10))

        for i, text in enumerate(lines):
            surf = self._font_sm.render(text, True, C["hud_text"])
            self._screen.blit(surf, (10 + padding, 10 + padding + i * line_h))

        # Key-binding hint bar at bottom
        hint = "ESC:quit  T:team  C:clear target  SPC:pause  LClick:select / set target  RClick:deselect"
        hint_surf = self._font_sm.render(hint, True, (180, 180, 180))
        sw, sh = self._screen.get_size()
        self._screen.blit(hint_surf, (sw // 2 - hint_surf.get_width() // 2, sh - 20))

    # ------------------------------------------------------------------
    # Drawing helpers
    # ------------------------------------------------------------------

    def _draw_dashed_polyline(
        self,
        points: list[tuple[int, int]],
        colour: tuple[int, int, int],
        width:  int = 1,
    ) -> None:
        """Draw a dashed polyline through *points* on the screen surface."""
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg_len = math.hypot(dx, dy)
            if seg_len < 1:
                continue
            ux, uy = dx / seg_len, dy / seg_len
            pos = 0.0
            drawing = True
            while pos < seg_len:
                seg_end = min(pos + (self._DASH_LEN if drawing else self._DASH_GAP), seg_len)
                if drawing:
                    sx0 = int(x0 + ux * pos)
                    sy0 = int(y0 + uy * pos)
                    sx1 = int(x0 + ux * seg_end)
                    sy1 = int(y0 + uy * seg_end)
                    pygame.draw.line(self._screen, colour, (sx0, sy0), (sx1, sy1), width)
                pos = seg_end
                drawing = not drawing

    @staticmethod
    def _draw_arrowhead(
        surf:   pygame.Surface,
        colour: tuple[int, int, int],
        start:  tuple[int, int],
        end:    tuple[int, int],
        size:   int = 10,
    ) -> None:
        """Draw a filled arrowhead at *end* pointing away from *start*."""
        dx, dy = end[0] - start[0], end[1] - start[1]
        mag = math.hypot(dx, dy)
        if mag < 1:
            return
        ux, uy = dx / mag, dy / mag
        # Two base corners perpendicular to direction
        px, py = -uy, ux
        tip   = end
        base1 = (int(end[0] - ux * size + px * size * 0.5),
                 int(end[1] - uy * size + py * size * 0.5))
        base2 = (int(end[0] - ux * size - px * size * 0.5),
                 int(end[1] - uy * size - py * size * 0.5))
        pygame.draw.polygon(surf, colour, [tip, base1, base2])


    @staticmethod
    def _draw_mini_arrowhead(surf, color, start, end):
        """Desenha uma ponta de seta pequena para vetores de trajetória."""
        dx, dy = end[0] - start[0], end[1] - start[1]
        dist = math.hypot(dx, dy)
        if dist < 5: return  # Muito pequeno para desenhar ponta

        # Vetor unitário reverso
        ux, uy = dx / dist, dy / dist

        # Tamanho da ponta
        size = 5

        # Pontos da base da ponta da seta
        # Rotaciona 150 graus para as "asas" da seta
        angle = math.radians(150)
        for a in [angle, -angle]:
            rx = ux * math.cos(a) - uy * math.sin(a)
            ry = ux * math.sin(a) + uy * math.cos(a)
            p_base = (end[0] + rx * size, end[1] + ry * size)
            pygame.draw.line(surf, color, end, p_base, 2)

# ---------------------------------------------------------------------------
# LCM publisher helper
# ---------------------------------------------------------------------------

def _publish_target(lc: object, robot_id: int, x_m: float, y_m: float) -> None:
    """
    Publish a TARGETS message.

    Parameters
    ----------
    lc:       lcm.LCM instance (or None in offline mode)
    robot_id: robot to command
    x_m, y_m: target position in **metres**
    """
    payload = json.dumps({"robot_id": robot_id, "x": x_m, "y": y_m}).encode()
    if lc is not None and _LCM_AVAILABLE:
        lc.publish("TARGETS", payload)
        print(f"[TARGETS] robot={robot_id}  x={x_m:.3f}  y={y_m:.3f} m")
    else:
        print(f"[TARGETS (offline)] robot={robot_id}  x={x_m:.3f}  y={y_m:.3f} m")


# ---------------------------------------------------------------------------
# Demo / stub data generators (used when LCM / data package unavailable)
# ---------------------------------------------------------------------------

def _inject_demo_state(shared: SharedState, t: float) -> None:
    """Populate SharedState with animated demo data for offline testing."""
    robots: dict[int, RobotState] = {}
    for i in range(4):
        angle = 2 * math.pi * i / 4 + t * 0.3
        robots[i] = RobotState(
            robot_id=i,
            x=2000 * math.cos(angle),
            y=2000 * math.sin(angle),
            orientation=angle + math.pi,
            team="yellow",
            vel_tang=1.5,
            vel_normal=0.5,
        )
    for i in range(3):
        angle = 2 * math.pi * i / 3 - t * 0.25
        key = -(i + 1)
        robots[key] = RobotState(
            robot_id=i,
            x=1500 * math.cos(angle),
            y=1500 * math.sin(angle),
            orientation=angle,
            team="blue",
            vel_tang=1.0,
            vel_normal=-0.3,
        )
    ball = BallState(
        x=600 * math.sin(t * 0.7),
        y=400 * math.cos(t * 0.5),
    )
    # Demo path for robot 0
    path_pts = [
        Point(robots[0].x, robots[0].y),
        Point(robots[0].x + 500, robots[0].y + 500),
        Point(0, 0),
    ]

    ia: dict[int, tuple[float, float]] = {k: (1.5, 0.3) for k in robots}

    with shared.lock:
        shared.robots      = robots
        shared.ball        = ball
        shared.paths       = {0: path_pts}
        shared.timestamp   = t
        shared.ia_commands = ia


# ---------------------------------------------------------------------------
# Hit-test helper
# ---------------------------------------------------------------------------

def _robot_at_pixel(
    px: int,
    py: int,
    robots: dict[int, RobotState],
    tf:     CoordTransform,
) -> Optional[int]:
    """Return the key of the robot whose circle contains pixel (px, py), or None."""
    r_px = tf.robot_radius_px()
    for key, robot in robots.items():
        cx, cy = tf.mm_to_px(robot.x, robot.y)
        if math.hypot(px - cx, py - cy) <= r_px:
            return key
    return None


# ---------------------------------------------------------------------------
# Main application loop
# ---------------------------------------------------------------------------

def main() -> None:  # noqa: C901
    """
    Entry point.  Initialises PyGame, starts the LCM thread, and runs
    the event / render loop at 60 fps.
    """
    pygame.init()

    WIN_W, WIN_H = 1280, 800
    screen = pygame.display.set_mode(
        (WIN_W, WIN_H),
        pygame.RESIZABLE,
    )
    pygame.display.set_caption("SSL Robot Soccer — Debug Visualizer")
    clock  = pygame.time.Clock()

    # ------------------------------------------------------------------
    # Shared state & LCM
    # ------------------------------------------------------------------
    shared = SharedState()

    lcm_thread = LCMThread(shared)
    lcm_thread.start()

    lc_pub: Optional[object] = None
    if _LCM_AVAILABLE:
        import lcm as _lcm
        lc_pub = _lcm.LCM()

    planner_thread = PlannerThread(shared, team_blue=False, lc_pub=lc_pub)
    planner_thread.start()

    # ------------------------------------------------------------------
    # Render / interaction state
    # ------------------------------------------------------------------
    renderer     = Renderer(screen)
    selected_key: Optional[int] = None
    active_team  = "yellow"
    paused       = False
    demo_mode    = not (_LCM_AVAILABLE and _DATA_AVAILABLE)
    t0           = time.time()

    print(f"[main] demo_mode={demo_mode}  LCM={_LCM_AVAILABLE}  data={_DATA_AVAILABLE}")

    running = True
    while running:
        # ── 1. Events ─────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_t:
                    active_team = "blue" if active_team == "yellow" else "yellow"
                    selected_key = None
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    with shared.lock:
                        shared.paused = paused
                elif event.key == pygame.K_c:
                    # Clear target for the selected robot
                    if selected_key is not None:
                        with shared.lock:
                            snap = deepcopy(shared.robots)
                        if selected_key in snap:
                            rid = snap[selected_key].robot_id
                            with shared.lock:
                                shared.targets.pop(rid, None)
                                shared.ia_commands.pop(rid, None)
                                shared.paths.pop(selected_key, None)
                            print(f"[main] Cleared target for robot {rid}")

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos

                # Snapshot geometry for coordinate transforms
                with shared.lock:
                    geom_snap  = deepcopy(shared.geometry)
                    robots_snap = deepcopy(shared.robots)

                win_w, win_h = screen.get_size()
                tf = CoordTransform(win_w, win_h, geom_snap)

                if event.button == 1:  # Left click
                    hit = _robot_at_pixel(mx, my, robots_snap, tf)
                    if hit is not None:
                        # Select the robot only if it belongs to the active team
                        r = robots_snap[hit]
                        if r.team == active_team:
                            selected_key = hit
                    else:
                        # Field click → set target for selected robot
                        if selected_key is not None and selected_key in robots_snap:
                            x_mm, y_mm = tf.px_to_mm(mx, my)
                            x_m, y_m = x_mm / 1000.0, y_mm / 1000.0
                            rid = robots_snap[selected_key].robot_id
                            # Write target directly into SharedState for PlannerThread
                            with shared.lock:
                                shared.targets[rid] = Point(x_m, y_m)
                            # Also publish TARGETS over LCM for external consumers
                            _publish_target(lc_pub, rid, x_m, y_m)

                elif event.button == 3:  # Right click → deselect
                    selected_key = None

            elif event.type == pygame.VIDEORESIZE:
                # pygame.RESIZABLE handles surface update automatically in
                # modern pygame; nothing extra needed here.
                pass

        # ── 2. Demo injection ─────────────────────────────────────────
        if demo_mode and not paused:
            _inject_demo_state(shared, time.time() - t0)

        # ── 3. Snapshot shared state (hold lock briefly) ───────────────
        with shared.lock:
            robots_f    = deepcopy(shared.robots)
            ball_f      = deepcopy(shared.ball)
            geom_f      = deepcopy(shared.geometry)
            paths_f     = deepcopy(shared.paths)
            ia_cmds_f   = deepcopy(shared.ia_commands)
            targets_f   = deepcopy(shared.targets)
            timestamp_f = shared.timestamp

        # ── 4. Render ─────────────────────────────────────────────────
        fps = clock.get_fps()
        renderer.draw_frame(
            robots       = robots_f,
            ball         = ball_f,
            geom         = geom_f,
            paths        = paths_f,
            ia_commands  = ia_cmds_f,
            targets      = targets_f,
            selected_key = selected_key,
            active_team  = active_team,
            timestamp    = timestamp_f,
            paused       = paused,
            fps          = fps,
        )

        pygame.display.flip()
        clock.tick(60)

    # ── Shutdown ───────────────────────────────────────────────────────
    print("[main] shutting down…")
    planner_thread.stop()
    planner_thread.join(timeout=2.0)
    lcm_thread.stop()
    lcm_thread.join(timeout=2.0)
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()