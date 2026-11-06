"""Parser and pulse round-trip for tools/compare_pulses.py."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

from app.pulse_config import CELL_PULSE, MA2010_PULSES_PER_DEGREE, joint_pulses_to_deg


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).parent / "fixtures" / "robot-cell"


def _load_tool():
    path = ROOT / "tools" / "compare_pulses.py"
    spec = importlib.util.spec_from_file_location("compare_pulses", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


cp = _load_tool()


def test_parse_production_000_job():
    job = cp.parse_jbi_file(FIXTURES / "000.JBI")
    assert job.name == "000"
    assert job.npos == "20,20,20,0,0,0"
    assert job.tool == 0
    assert job.postype == "PULSE"
    assert job.group1 == "RB1,BS1"
    assert job.group2 == "ST1"
    assert len(job.points) == 20
    assert job.points[0].c == [95902, -19559, -65353, -68228, -62443, 70439]
    assert job.points[0].bc == 878036
    assert job.points[0].ec == [-1746, 0]
    assert job.points[19].c == [116865, -4881, -33684, -3254, -73880, -61311]
    opcodes = [inst.opcode for inst in job.instructions]
    for required in ("MOVJ", "MOVL", "MOVC", "ARCON", "ARCOF", "WVON", "MACRO1"):
        assert required in opcodes
    assert opcodes[0].startswith("MOVJ")


def test_parse_home_track_and_two_axis_ec():
    job = cp.parse_jbi_file(FIXTURES / "HOME_TRACK.JBI")
    home = [-283, -101162, -84213, 3710, -80188, -1936]
    assert job.points[0].c == home
    assert job.points[1].c == home
    assert job.points[0].bc == 570125
    assert job.points[0].ec == [192, 0]
    assert job.points[1].ec == [-2638, -94527]
    assert {inst.opcode for inst in job.instructions} == {"MOVJ"}


def test_home_pulses_match_pulse_config_scale():
    pulses = [-283, -101162, -84213, 3710, -80188, -1936]
    deg = cp.pulses_to_degrees(pulses)
    assert deg == pytest.approx(joint_pulses_to_deg(pulses))
    assert deg[0] == pytest.approx(pulses[0] / MA2010_PULSES_PER_DEGREE[0])
    back = cp.degrees_to_pulses(deg)
    assert back == pulses
    assert cp.ec_axis1_deg([192, 0]) == pytest.approx(192 / CELL_PULSE.station_pulses_per_degree)
    assert cp.softlimit_axes(pulses) == []


def test_directory_of_jbi_files():
    paths = cp.iter_jbi_paths(FIXTURES)
    names = {p.name for p in paths}
    assert "000.JBI" in names
    assert "HOME_TRACK.JBI" in names


def test_round_trip_reports_tcp_and_deltas():
    job = cp.parse_jbi_file(FIXTURES / "HOME_TRACK.JBI")
    rows = cp.compare_job(job, do_ik=True)
    assert len(rows) == 2
    row = rows[0]
    assert row.pulses[0] == -283
    assert row.bc == 570125
    assert set(row.tcp) == {"x", "y", "z", "rx", "ry", "rz"}
    assert row.delta_pulses is not None and len(row.delta_pulses) == 6
    assert row.delta_degrees is not None and len(row.delta_degrees) == 6
    assert row.ik_ok is True
    # Seeded with the taught pose, the numeric IK should come back to the same pulses.
    assert max(abs(v) for v in row.delta_pulses) <= 2
    text = cp.format_job(job, rows)
    assert "HOME_TRACK" in text
    assert "GROUP1=RB1,BS1" in text
    assert "C00000" in text
    assert "570125" in text


def test_rectan_job_is_rejected():
    text = (FIXTURES.parent / "DROP01.JBI").read_text(encoding="utf-8")
    job = cp.parse_jbi(text, source="DROP01.JBI")
    assert job.postype == "USER"
    with pytest.raises(ValueError, match="POSTYPE"):
        cp.compare_job(job, do_ik=False)


def test_empty_npos_job():
    path = FIXTURES / "EMPTY.JBI"
    if not path.is_file():
        path = FIXTURES / "1-DEFRO-60.JBI"
    if not path.is_file():
        pytest.skip("empty NPOS fixture missing")
    job = cp.parse_jbi_file(path)
    assert job.npos and job.npos.startswith("0,")
    rows = cp.compare_job(job, do_ik=False)
    assert rows == []


def test_multipoint_000_and_optional_kosz():
    job = cp.parse_jbi_file(FIXTURES / "000.JBI")
    rows = cp.compare_job(job, do_ik=False)
    assert len(rows) == 20
    assert all(r.limit_flag in {"OK", "LIMIT_WARN", "OUT_OF_RANGE"} for r in rows)

    kosz = FIXTURES / "02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI"
    if not kosz.is_file():
        return
    big = cp.parse_jbi_file(kosz)
    n_c = sum(1 for p in big.points.values() if p.c and len(p.c) == 6)
    assert n_c == 218
    assert len(cp.compare_job(big, do_ik=False)) == 218


def test_markdown_out(tmp_path):
    out = tmp_path / "report.md"
    code = cp.main([str(FIXTURES / "HOME_TRACK.JBI"), "--no-ik", "--markdown-out", str(out)])
    assert code == 0
    text = out.read_text(encoding="utf-8")
    assert "PULSE compare report" in text
    assert "HOME_TRACK" in text
    assert "C00000" in text


def test_golden_poses_fixture_loaded():
    data = cp.load_golden_poses()
    assert data["schema"].startswith("yaskawa-cadcam/golden_poses")
    ids = [p["id"] for p in data["poses"]]
    assert ids == [
        "POSE_01_SAFE_HOME",
        "POSE_02_PARK_000",
        "POSE_03_ST1_WELD_START",
        "POSE_04_ST2_KOSZ_WELD_1",
        "POSE_05_ST2_KOSZ_WELD_MID",
        "POSE_06_ST2_ARC_MOVC",
        "POSE_07_ST3_ROTARY_WELD",
        "POSE_08_ST4_WELD",
    ]
    for pose in data["poses"]:
        assert pose["verified_on_pendant"] is False
        assert pose["cartesian_status"] == "UNVERIFIED_ON_PENDANT"
        assert pose["pendant_or_motosim"] is None
        assert len(pose["pulses"]) == 6


def test_check_golden_cli():
    code, report = cp.check_golden(do_ik=True)
    assert code == 0
    assert "POSE_01_SAFE_HOME" in report
    assert "UNVERIFIED_ON_PENDANT" in report
    assert "errors=0" in report
    assert cp.main(["--check-golden"]) == 0


def test_check_golden_rejects_fake_pendant(tmp_path):
    data = cp.load_golden_poses()
    data["poses"][0]["pendant_or_motosim"] = {
        "x_mm": 0.0,
        "y_mm": 0.0,
        "z_mm": 0.0,
        "rx_deg": 0.0,
        "ry_deg": 0.0,
        "rz_deg": 0.0,
    }
    # still verified_on_pendant=false → honesty error
    bad = tmp_path / "bad_golden.json"
    bad.write_text(json.dumps(data), encoding="utf-8")
    code, report = cp.check_golden(bad, do_ik=False)
    assert code == 1
    assert "verified_on_pendant=false" in report
