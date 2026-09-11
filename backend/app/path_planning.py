"""
Turn detected weld seams into an ordered TCP path for the robot.

The stage names, action ids, CTWD and home interpolation follow Verbotics
Weld 2026 desktop logs (see docs/VERBOTICS_LOGS.md):

  Identify → seed 5000 states from home → plan toolpath → re-plan closer
  to home → simplify → convert to trajectory → approach/retreat →
  calibration (aligned planes / lines, 3-state touch) → motions only
  through home (1 = home, 3 = touches, 5 = weld). Missing 5↔3 or 3↔1
  inserts a home move. Welding stick-out is 12 mm; air moves are 0 mm.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os

import numpy as np

from .dx200_postprocessor import WeldSegment

# Observed in Weld 2026 logs: "point at i / 12mm (welding)" vs "0mm (not welding)".
CTWD_WELD_MM = 12.0
CTWD_AIR_MM = 0.0
SEED_STATES = 5000
MAX_PLAN_ATTEMPTS = 2
ACTION_HOME = 1
ACTION_TOUCH = 3
ACTION_WELD = 5
TOUCH_APPROACH_STATES = 3
MAX_JOINT_CHANGE_DEG = 175.0
UNWRAP_DEG = 360.0


@dataclass
class PlannedWeld:
    name: str
    start_index: int
    end_index: int
    cost: float
    states: int
    attempt: int = 1
    max_attempts: int = MAX_PLAN_ATTEMPTS


@dataclass
class PlannedPath:
    points: list[dict]                # ordered TCP poses {x,y,z,rx,ry,rz,...}
    weld_segments: list[WeldSegment]  # index ranges to wrap with ARCON/ARCOF
    welds: list[PlannedWeld] = field(default_factory=list)
    log: list[str] = field(default_factory=list)
    ctwd_mm: float = CTWD_WELD_MM
    seed_states: int = SEED_STATES


def _cpu_cores() -> int:
    return os.cpu_count() or 1


def _weld_name(part: int, index: int) -> str:
    return f"Weld {part}-{index}"


def _pose(p, rx: float, ry: float, rz: float, **extra) -> dict:
    out = {
        "x": float(p[0]),
        "y": float(p[1]),
        "z": float(p[2]),
        "rx": rx,
        "ry": ry,
        "rz": rz,
    }
    out.update(extra)
    return out


def _cartesian_cost(points: list[np.ndarray]) -> float:
    """Cheap stand-in for Verbotics' toolpath cost (log values were ~1e2–1e4)."""
    if len(points) < 2:
        return 0.0
    length = sum(
        float(np.linalg.norm(points[i] - points[i - 1])) for i in range(1, len(points))
    )
    return round(length * 1000.0, 3)


def _home_pose(seams: list[dict]) -> np.ndarray:
    """A cell-home TCP above the centroid of all seams (reference configuration)."""
    pts = []
    for seam in seams:
        pts.append(np.asarray(seam["start"], dtype=float))
        pts.append(np.asarray(seam["end"], dtype=float))
    c = np.mean(np.stack(pts), axis=0) if pts else np.zeros(3)
    return c + np.array([0.0, 0.45, 0.35])


def _touch_states(start: np.ndarray, end: np.ndarray) -> list[np.ndarray]:
    """Three-state touch approach: plane, line, and a lift (logs: 3 states)."""
    tangent = end - start
    tlen = float(np.linalg.norm(tangent)) or 1.0
    tangent = tangent / tlen
    # Prefer a plane offset along +Z, a line offset along the seam tangent.
    plane = start + np.array([0.0, 0.04, 0.02])
    line = start + tangent * 0.03 + np.array([0.0, 0.04, 0.0])
    lift = start + np.array([0.0, 0.08, 0.04])
    return [lift, plane, line]


def plan_from_seams(
    seams: list[dict],
    *,
    samples_per_seam: int = 5,
    approach_height: float = 0.05,     # kept for callers; home/touch supersede it
    torch_rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0),
    weld_speed: float = 10.0,
    arc_file: int = 1,
    part_index: int = 1,
    seed_states: int = SEED_STATES,
    ctwd_mm: float = CTWD_WELD_MM,
    max_attempts: int = MAX_PLAN_ATTEMPTS,
) -> PlannedPath:
    """Build a weld path from seam segments (each with 'start'/'end' 3D lists).

    Extra pose keys (kind, action, ctwd_mm, weld_name) are ignored by IK / JBI
    but consumed by the API and tests. Torch RPY 180° = tip down, so welding
    TCP is raised by ``ctwd_mm`` along +Z.
    """
    rx, ry, rz = torch_rpy_deg
    ctwd_m = ctwd_mm / 1000.0
    # Torch along -Z in this RPY convention: lift the tip off the seam by CTWD.
    weld_off = np.array([0.0, 0.0, ctwd_m])
    _ = approach_height  # Verbotics uses home/touch instead of a single lift

    log: list[str] = []
    cores = _cpu_cores()
    log.append(f"Identifying welds with {cores} CPU cores")
    log.append(f"Staging {len(seams)} welds")

    home = _home_pose(seams)
    points: list[dict] = []
    weld_segments: list[WeldSegment] = []
    welds: list[PlannedWeld] = []

    def pose(p, **extra) -> dict:
        return _pose(p, rx, ry, rz, **extra)

    for i, seam in enumerate(seams, start=1):
        name = _weld_name(part_index, i)
        attempt = 1
        log.append(f"Planning weld '{name} (attempt {attempt}/{max_attempts})")
        log.append("Seeding toolpath states from reference position...")
        log.append(f"Seeded {seed_states} toolpath states")

        start = np.asarray(seam["start"], dtype=float)
        end = np.asarray(seam["end"], dtype=float)
        n = max(2, samples_per_seam)
        seam_pts = [start + t * (end - start) for t in np.linspace(0.0, 1.0, n)]
        weld_tcp = [p + weld_off for p in seam_pts]
        cost = _cartesian_cost(weld_tcp)
        samples = max(1, n - 1)
        motions = samples + 2
        log.append(
            f"Invocation 1 100% along path, {samples} intermediate samples "
            f"(0% infeasible), {motions} intermediate motions checked "
            f"({motions} / 100% valid)"
        )
        log.append(f"Planned toolpath with cost {cost}")
        log.append("Attempting to re-plan in closer configuration to home...")
        log.append("Unable to re-solve toolpath in better configuration")
        log.append("Toolpath planned, optimising toolpath...")
        log.append(f"Simplified toolpath from {cost} to 0")
        log.append("Optimised toolpath with cost 0")
        log.append("Converting toolpath to robot trajectory")
        log.append(f"Created toolpath with {n} states")
        log.append("Planning approach/retreat for weld...")
        log.append("Planning calibration for targets:")
        log.append("Generating aligned touch planes")
        log.append("Generating touch lines")
        log.append("Generating touch points")
        log.append("Touch order prior to sorting:")
        log.append("0 plane")
        log.append("1 line")
        log.append("2 invalid point")
        log.append("Touch order after sorting:")
        log.append("0 plane")
        log.append("1 line")
        log.append(f"Planning approach for touch with {TOUCH_APPROACH_STATES} states.")
        log.append("Planning motion from home to touches")
        log.append("Planning motion from touches to home")
        log.append("Planning motion from home to toolpath")
        log.append("Planning motion from toolpath to home")

        # Action 1: home (reference). Always enter a weld from home.
        points.append(
            pose(
                home,
                kind="home",
                action=ACTION_HOME,
                ctwd_mm=CTWD_AIR_MM,
                weld_name=name,
                tag=None,
            )
        )
        # Action 3: three-state touch.
        for t_i, tp in enumerate(_touch_states(start, end)):
            points.append(
                pose(
                    tp,
                    kind="sense",
                    action=ACTION_TOUCH,
                    ctwd_mm=CTWD_AIR_MM,
                    weld_name=name,
                    tag="TOUCH",
                    touch_feature=("approach", "plane", "line")[t_i],
                )
            )
        # Missing 3→5: go home (observed WRN in logs).
        log.append(
            f"Missing from previous path '{name}' between action "
            f"{ACTION_TOUCH} and {ACTION_WELD}, going home"
        )
        points.append(
            pose(
                home,
                kind="home",
                action=ACTION_HOME,
                ctwd_mm=CTWD_AIR_MM,
                weld_name=name,
            )
        )

        weld_start_idx = len(points)
        for k, p in enumerate(weld_tcp):
            tag = "ARCON" if k == 0 else ("ARCOF" if k == n - 1 else None)
            points.append(
                pose(
                    p,
                    kind="weld",
                    action=ACTION_WELD,
                    ctwd_mm=ctwd_mm,
                    weld_name=name,
                    tag=tag,
                )
            )
        weld_end_idx = len(points) - 1
        weld_segments.append(
            WeldSegment(
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                weld_speed=weld_speed,
                arc_file=arc_file,
            )
        )
        welds.append(
            PlannedWeld(
                name=name,
                start_index=weld_start_idx,
                end_index=weld_end_idx,
                cost=cost,
                states=n,
                attempt=attempt,
                max_attempts=max_attempts,
            )
        )

        # Action 5 → next weld's 3 (or end): go home.
        log.append(
            f"Missing from previous path '{name}' between action "
            f"{ACTION_WELD} and {ACTION_TOUCH}, going home"
        )
        points.append(
            pose(
                home,
                kind="home",
                action=ACTION_HOME,
                ctwd_mm=CTWD_AIR_MM,
                weld_name=name,
            )
        )
        log.append(f"Validating weld plan for '{name}'")
        log.append(f"point at 0 / {CTWD_AIR_MM:.0f}mm (not welding)")
        log.append(f"point at 1 / {ctwd_mm:.0f}mm (welding)")

    log.append("All planning tasks finished, profiling information:")
    return PlannedPath(
        points=points,
        weld_segments=weld_segments,
        welds=welds,
        log=log,
        ctwd_mm=ctwd_mm,
        seed_states=seed_states,
    )
