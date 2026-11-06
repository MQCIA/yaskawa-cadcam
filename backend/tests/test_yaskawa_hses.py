"""UnderAutomation HSES adapter — dry-run / probe tests (no robot required)."""

from __future__ import annotations

import os

import pytest

from app.yaskawa_hses import (
    describe_hses_link,
    get_status,
    select_and_start_job,
    sdk_probe,
    upload_and_start,
    upload_jbi,
)


@pytest.fixture(autouse=True)
def _dry_run(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("YASKAWA_UA_DRY_RUN", "1")
    monkeypatch.setenv("YASKAWA_ROBOT_HOST", "192.168.0.50")


def test_describe_hses_link():
    info = describe_hses_link()
    assert info["provider"] == "UnderAutomation.Yaskawa"
    assert info["dry_run"] is True
    assert info["host"] == "192.168.0.50"
    assert "sdk" in info


def test_sdk_probe_shape():
    p = sdk_probe()
    assert "available" in p
    assert p["package"] == "UnderAutomation.Yaskawa"


def test_dry_run_status():
    st = get_status()
    assert st["dry_run"] is True
    assert "servo_on" in st


def test_dry_run_upload_and_start():
    jbi = "/JOB\n//NAME CADCAM\nNOP\nEND\n"
    up = upload_jbi(jbi, job_name="DEMO_WELD.JBI")
    assert up["ok"] is True
    assert up["job"] == "DEMO_WELD"
    assert up["remote"] == "DEMO_WELD.JBI"

    start = select_and_start_job("DEMO_WELD", line=0)
    assert start["started"] is True

    both = upload_and_start(jbi, job_name="CADCAM")
    assert both["upload"]["ok"] is True
    assert both["start"]["started"] is True


def test_upload_fails_without_dry_run_config(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("YASKAWA_UA_DRY_RUN", "0")
    monkeypatch.delenv("YASKAWA_ROBOT_HOST", raising=False)
    with pytest.raises(RuntimeError):
        # Either missing host or missing .NET/SDK — both are RuntimeError.
        upload_jbi("/JOB\nNOP\nEND\n")
