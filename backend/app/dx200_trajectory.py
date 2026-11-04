"""
DX200-style motion interpolation & cycle-time calculator.

Separates joint-space ``MOVJ`` from Cartesian ``MOVL`` / ``MOVC`` / ``MOVS``,
applies a trapezoidal profile scaled by ``VJ`` and ``DEC``, overlays weave
(``WVON WEV#``), and flags wrist singularities / excessive joint rates.

Joint max speeds are Motoman MA/AR-class catalogue values (deg/s) used for
offline cycle estimates — validate against the cell's RC.PRM / MotoSim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from .kinematics_engine import forward_kinematics, tcp_se3
from .robot_config import AXIS_NAMES

# Catalogue max joint rates (deg/s) — MA2010 / AR2010 class.
JOINT_MAX_DEG_S: tuple[float, ...] = (197.0, 175.0, 175.0, 360.0, 360.0, 520.0)
BC_MAX_MMS = 1500.0  # RECT-X carriage estimate
EC_MAX_DEG_S = 90.0  # TURN station estimate
WRIST_SING_B_DEG = 5.0
JOINT_RATE_WARN_DEG_S = 120.0

MoveKind = Literal["MOVJ", "MOVL", "MOVC", "MOVS", "TIMER"]


@dataclass
class TrajSample:
    t: float
    pos: list[float]
    dir: list[float]
    q_deg: list[float]
    bc: float | None = None
    ec_deg: float | None = None
    move: MoveKind = "MOVJ"
    arc_on: bool = False
    weave_active: bool = False
    warnings: list[str] = field(default_factory=list)


@dataclass
class SegmentTiming:
    index: int
    move: MoveKind
    duration_s: float
    length_m: float = 0.0


def _norm(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3)
    return v / n


def trapezoid_s(u: float, dec_pct: float = 20.0) -> float:
    """Map normalised time u∈[0,1] through accel / cruise / DEC coast.

    ``DEC`` on DX200 is a deceleration parameter (jobs use ``DEC=20``). We treat
    it as the fraction of the move spent braking (and symmetrically accelerating).
    """
    u = float(np.clip(u, 0.0, 1.0))
    dec = float(np.clip(dec_pct, 1.0, 45.0)) / 100.0
    acc = dec
    cruise = max(0.0, 1.0 - acc - dec)
    if u < acc and acc > 1e-9:
        # Quadratic ease-in → s = u²/(2·acc) scaled so total s(1)=1.
        # Use smoothstep-like S-curve for simplicity matching pendant feel.
        x = u / acc
        return 0.5 * acc * (x * x)
    if u < acc + cruise:
        # Linear cruise: continue from s(acc)=0.5·acc
        s0 = 0.5 * acc
        return s0 + (u - acc)
    # Decel
    x = (u - (acc + cruise)) / max(dec, 1e-9)
    s_cruise_end = 0.5 * acc + cruise
    # Ease-out so s(1)=1
    return s_cruise_end + dec * (x - 0.5 * x * x)


def _normalize_profile(u: float, dec_pct: float = 20.0) -> float:
    """Trapezoid_s scaled so s(0)=0 and s(1)=1."""
    raw = trapezoid_s(u, dec_pct)
    end = trapezoid_s(1.0, dec_pct)
    if end < 1e-12:
        return u
    return float(np.clip(raw / end, 0.0, 1.0))


def movj_duration_s(
    q0: list[float],
    q1: list[float],
    *,
    vj: float = 80.0,
    dec: float = 20.0,
    bc0: float | None = None,
    bc1: float | None = None,
    ec0: float | None = None,
    ec1: float | None = None,
) -> float:
    """t = max_i |Δq_i| / (ω_max_i · VJ/100), plus BC / EC axes."""
    vj_frac = max(float(vj), 0.1) / 100.0
    # DEC slightly stretches the move (lower average speed).
    dec_stretch = 1.0 + 0.35 * (float(dec) / 100.0)
    t_axes = []
    for i, (a, b) in enumerate(zip(q0, q1)):
        wmax = JOINT_MAX_DEG_S[i] * vj_frac
        t_axes.append(abs(b - a) / max(wmax, 1e-6))
    if bc0 is not None and bc1 is not None:
        # Treat BC as linear metres if |Δ|<20 else pulses≈0.002 mm/pulse heuristic.
        dbc = abs(bc1 - bc0)
        if dbc > 50:
            dbc_m = dbc * 0.002 / 1000.0  # ~2 µm/pulse placeholder
        else:
            dbc_m = dbc
        t_axes.append(dbc_m / max((BC_MAX_MMS / 1000.0) * vj_frac, 1e-6))
    if ec0 is not None and ec1 is not None:
        t_axes.append(abs(ec1 - ec0) / max(EC_MAX_DEG_S * vj_frac, 1e-6))
    return float(max(t_axes) * dec_stretch) if t_axes else 0.0


def cartesian_duration_s(length_m: float, v_cm_min: float) -> float:
    """t = length_mm / V_mm_s  with V from cm/min."""
    v_mms = max(float(v_cm_min), 0.01) * 10.0 / 60.0  # cm/min → mm/s
    return (max(length_m, 0.0) * 1000.0) / v_mms


def circle_through_3(
    p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
) -> tuple[np.ndarray, float, np.ndarray, np.ndarray, np.ndarray, float, float] | None:
    """Return centre, radius, u, v, normal, angle0, angle_span for arc P1→P2→P3."""
    a, b, c = np.asarray(p1, float), np.asarray(p2, float), np.asarray(p3, float)
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    if float(np.linalg.norm(n)) < 1e-10:
        return None
    ab2, ac2, abac = float(np.dot(ab, ab)), float(np.dot(ac, ac)), float(np.dot(ab, ac))
    den = 2.0 * (ab2 * ac2 - abac * abac)
    if abs(den) < 1e-14:
        return None
    k1 = (ab2 * ac2 - ac2 * abac) / den
    k2 = (ac2 * ab2 - ab2 * abac) / den
    centre = a + k1 * ab + k2 * ac
    r = float(np.linalg.norm(a - centre))
    if not math.isfinite(r) or r < 1e-6:
        return None
    nn = _norm(n)
    u = _norm(a - centre)
    v = _norm(np.cross(nn, u))

    def ang(p: np.ndarray) -> float:
        d = _norm(p - centre)
        return math.atan2(float(np.dot(d, v)), float(np.dot(d, u)))

    a0, a1, a2 = ang(a), ang(b), ang(c)

    def unwrap(prev: float, nxt: float) -> float:
        x = nxt
        while x - prev > math.pi:
            x -= 2 * math.pi
        while x - prev < -math.pi:
            x += 2 * math.pi
        return x

    a1 = unwrap(a0, a1)
    a2 = unwrap(a1, a2)
    return centre, r, u, v, nn, a0, a2 - a0


def sample_circle(
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    s: float,
) -> np.ndarray | None:
    """Point at normalised arc parameter s∈[0,1] on circle through 3 points."""
    circ = circle_through_3(p1, p2, p3)
    if circ is None:
        return (1 - s) * np.asarray(p1, float) + s * np.asarray(p3, float)
    centre, r, u, v, _n, a0, span = circ
    # Parameterise via quadratic angle through mid so s=0.5 ≈ P2.
    ang = (1 - s) * (1 - s) * a0 + 2 * (1 - s) * s * (a0 + span / 2) + s * s * (a0 + span)
    return centre + r * (math.cos(ang) * u + math.sin(ang) * v)


def slerp_dir(d0: np.ndarray, d1: np.ndarray, s: float) -> np.ndarray:
    a = _norm(d0)
    b = _norm(d1)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot > 0.9995:
        return _norm((1 - s) * a + s * b)
    omega = math.acos(dot)
    so = math.sin(omega)
    return _norm((math.sin((1 - s) * omega) * a + math.sin(s * omega) * b) / so)


def weave_offset(
    travel: np.ndarray,
    bisector: np.ndarray,
    t_sec: float,
    *,
    amp_mm: float = 3.0,
    freq_hz: float = 2.5,
    triangular: bool = False,
) -> np.ndarray:
    """Lateral weave in plane ⟂ travel, along travel×bisector."""
    tr = _norm(travel)
    bi = _norm(bisector)
    lat = _norm(np.cross(tr, bi))
    if float(np.linalg.norm(lat)) < 1e-8:
        lat = _norm(np.cross(tr, np.array([0.0, 1.0, 0.0])))
    phase = 2.0 * math.pi * freq_hz * t_sec
    if triangular:
        # Triangle wave ∈ [-1,1]
        u = (t_sec * freq_hz) % 1.0
        wave = 4.0 * abs(u - 0.5) - 1.0
    else:
        wave = math.sin(phase)
    return lat * (amp_mm / 1000.0) * wave


def wrist_singularity_warning(q_deg: list[float]) -> str | None:
    if abs(q_deg[4]) < WRIST_SING_B_DEG:  # B axis
        return "Uwaga: Osobliwość kiści (Oś B blisko 0°)"
    return None


def joint_rate_warnings(
    q_prev: list[float], q_now: list[float], dt: float
) -> list[str]:
    if dt < 1e-9:
        return []
    out = []
    for i, name in enumerate(AXIS_NAMES):
        rate = abs(q_now[i] - q_prev[i]) / dt
        if rate > JOINT_RATE_WARN_DEG_S and name in ("R", "T", "B"):
            out.append(f"Prędkość osi {name}: {rate:.0f}°/s (> {JOINT_RATE_WARN_DEG_S})")
    return out


def interpolate_movj_q(
    q0: list[float], q1: list[float], u: float, *, dec: float = 20.0
) -> list[float]:
    s = _normalize_profile(u, dec)
    return [a + (b - a) * s for a, b in zip(q0, q1)]


def fk_tcp_pos_dir(q_deg: list[float]) -> tuple[list[float], list[float]]:
    """TCP position + wire direction (−Z of Motoman tool file ≈ approach)."""
    T = tcp_se3(q_deg)
    pos = [float(x) for x in T.t]
    # Wire stick-out ≈ Motoman tool0 +Z after TOOL.CND = third column of R.
    R = np.array(T.R, dtype=float)
    wire = _norm(R[:, 2])
    # Into-work for welding demos often −wire if Z points out of tip along stick-out.
    direction = [-float(wire[0]), -float(wire[1]), -float(wire[2])]
    return pos, direction


def segment_duration_from_poses(
    *,
    move: MoveKind,
    p0: list[float],
    p1: list[float],
    p_mid: list[float] | None = None,
    q0: list[float] | None = None,
    q1: list[float] | None = None,
    vj: float = 80.0,
    v_cm_min: float = 5.0,
    dec: float = 20.0,
    timer_s: float = 0.0,
    bc0: float | None = None,
    bc1: float | None = None,
    ec0: float | None = None,
    ec1: float | None = None,
) -> float:
    if move == "TIMER" or timer_s > 0 and move == "TIMER":
        return max(timer_s, 0.0)
    dwell = max(timer_s, 0.0)
    if move == "MOVJ":
        if q0 is not None and q1 is not None:
            return dwell + movj_duration_s(
                q0, q1, vj=vj, dec=dec, bc0=bc0, bc1=bc1, ec0=ec0, ec1=ec1
            )
        # Fallback: treat as slow Cartesian with VJ scaling.
        length = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
        return dwell + cartesian_duration_s(length, max(v_cm_min, 1.0) * (vj / 10.0))
    if move == "MOVC" and p_mid is not None:
        circ = circle_through_3(np.asarray(p0), np.asarray(p_mid), np.asarray(p1))
        if circ is not None:
            _c, r, _u, _v, _n, _a0, span = circ
            length = abs(span) * r
        else:
            length = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
        return dwell + cartesian_duration_s(length, v_cm_min)
    length = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
    return dwell + cartesian_duration_s(length, v_cm_min)


def sample_movj_tcp(
    q0: list[float], q1: list[float], u: float, *, dec: float = 20.0
) -> TrajSample:
    q = interpolate_movj_q(q0, q1, u, dec=dec)
    pos, direction = fk_tcp_pos_dir(q)
    warns = []
    w = wrist_singularity_warning(q)
    if w:
        warns.append(w)
    return TrajSample(
        t=0.0, pos=pos, dir=direction, q_deg=q, move="MOVJ", warnings=warns
    )


def build_movj_polyline(
    q0: list[float], q1: list[float], *, samples: int = 24, dec: float = 20.0
) -> list[list[float]]:
    """True MOVJ TCP arc via FK(θ(t)) for 3D collision preview."""
    pts = []
    for i in range(samples):
        u = i / max(samples - 1, 1)
        s = sample_movj_tcp(q0, q1, u, dec=dec)
        pts.append(s.pos)
    return pts
