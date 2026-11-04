"""DX200 trajectory / cycle-time unit tests."""

from __future__ import annotations

import math

import numpy as np

from app.dx200_trajectory import (
    build_movj_polyline,
    cartesian_duration_s,
    circle_through_3,
    interpolate_movj_q,
    movj_duration_s,
    sample_circle,
    sample_movj_tcp,
    trapezoid_s,
    weave_offset,
    wrist_singularity_warning,
)


def test_trapezoid_endpoints():
    assert trapezoid_s(0.0) == 0.0
    end = trapezoid_s(1.0, 20.0)
    assert end > 0.5
    # Monotonic
    prev = 0.0
    for i in range(21):
        s = trapezoid_s(i / 20.0, 20.0)
        assert s + 1e-9 >= prev
        prev = s


def test_movj_duration_scales_with_vj():
    q0 = [0.0, -45.0, -45.0, 0.0, -80.0, 0.0]
    q1 = [40.0, -20.0, -30.0, 10.0, -60.0, 20.0]
    t80 = movj_duration_s(q0, q1, vj=80.0)
    t40 = movj_duration_s(q0, q1, vj=40.0)
    assert t40 > t80 * 1.5
    assert t80 > 0.05


def test_movj_tcp_polyline_is_not_a_chord():
    q0 = [0.0, -50.0, -40.0, 0.0, -70.0, 0.0]
    q1 = [60.0, -10.0, -20.0, 30.0, -40.0, 40.0]
    poly = build_movj_polyline(q0, q1, samples=16)
    assert len(poly) == 16
    p0 = np.asarray(poly[0])
    p1 = np.asarray(poly[-1])
    mid = np.asarray(poly[len(poly) // 2])
    chord_mid = 0.5 * (p0 + p1)
    # True joint-space path bows away from the Cartesian chord.
    assert float(np.linalg.norm(mid - chord_mid)) > 0.01


def test_movj_sample_has_six_joints():
    q0 = [0.0] * 6
    q1 = [10.0, -5.0, 5.0, -5.0, 5.0, -5.0]
    s = sample_movj_tcp(q0, q1, 0.5)
    assert len(s.q_deg) == 6
    assert len(s.pos) == 3
    mid_q = interpolate_movj_q(q0, q1, 0.5)
    assert abs(mid_q[0] - 5.0) < 1.0  # near linear after profile


def test_cartesian_duration_cm_min():
    # 100 mm at 5 cm/min = 600 mm/min wait: 5 cm/min = 50 mm/min → 100/50 = 2 min = 120 s
    t = cartesian_duration_s(0.1, 5.0)
    assert abs(t - 120.0) < 1e-6


def test_circle_through_3_right_angle():
    p1 = np.array([1.0, 0.0, 0.0])
    p2 = np.array([0.0, 1.0, 0.0])
    p3 = np.array([-1.0, 0.0, 0.0])
    circ = circle_through_3(p1, p2, p3)
    assert circ is not None
    centre, r, *_ = circ
    assert abs(r - 1.0) < 1e-6
    assert float(np.linalg.norm(centre)) < 1e-6
    mid = sample_circle(p1, p2, p3, 0.5)
    assert mid is not None
    assert abs(float(np.linalg.norm(mid)) - 1.0) < 0.05


def test_wrist_singularity():
    q = [0.0, -45.0, -45.0, 0.0, 2.0, 0.0]
    assert wrist_singularity_warning(q) is not None
    q[4] = -40.0
    assert wrist_singularity_warning(q) is None


def test_weave_offset_perpendicular():
    travel = np.array([1.0, 0.0, 0.0])
    bis = np.array([0.0, 0.0, 1.0])
    off = weave_offset(travel, bis, 0.125, amp_mm=4.0, freq_hz=2.0)
    # Lateral ≈ Y (sin(2π·2·0.125)=sin(π/2)=1 → ±4 mm)
    assert abs(off[0]) < 1e-9
    assert abs(off[2]) < 1e-9
    assert abs(off[1]) > 1e-4
