"""
Optional live DX200 link via UnderAutomation Yaskawa SDK (HSES / FTP).

Complements — does **not** replace — the offline CAD/CAM path:
  plan seams → generate ``.JBI`` → (this module) upload + SelectJob + StartJob.

Commercial SDK: https://github.com/underautomation/Yaskawa.py
  ``pip install UnderAutomation.Yaskawa``
  Linux/macOS: install .NET 8 runtime and ``export PYTHONNET_RUNTIME=coreclr``
  Trial: 30 days; register with ``YASKAWA_UA_LICENSE_COMPANY`` + ``YASKAWA_UA_LICENSE_KEY``.

Env:
  ``YASKAWA_ROBOT_HOST`` — controller IP (required for live calls)
  ``YASKAWA_UA_DRY_RUN=1`` — simulate success without contacting the robot
  ``YASKAWA_UA_LICENSE_COMPANY`` / ``YASKAWA_UA_LICENSE_KEY`` — unlock trial
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Any


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


def sdk_probe() -> dict[str, Any]:
    """Report whether the commercial SDK can be imported in this process."""
    try:
        # Prefer CoreCLR on Linux before the package auto-loads Mono.
        if os.name != "nt" and not os.environ.get("PYTHONNET_RUNTIME"):
            os.environ.setdefault("PYTHONNET_RUNTIME", "coreclr")
        from underautomation.yaskawa.yaskawa_robot import YaskawaRobot  # noqa: F401

        return {
            "available": True,
            "package": "UnderAutomation.Yaskawa",
            "runtime": os.environ.get("PYTHONNET_RUNTIME") or "default",
            "hint": None,
        }
    except Exception as exc:  # noqa: BLE001 — surface any CLR / import failure
        return {
            "available": False,
            "package": "UnderAutomation.Yaskawa",
            "runtime": os.environ.get("PYTHONNET_RUNTIME") or "unset",
            "hint": (
                "pip install UnderAutomation.Yaskawa && "
                "install .NET 8 runtime, then: export PYTHONNET_RUNTIME=coreclr"
            ),
            "error": f"{type(exc).__name__}: {exc}",
        }


def describe_hses_link() -> dict[str, Any]:
    host = os.environ.get("YASKAWA_ROBOT_HOST", "").strip()
    dry = _env_bool("YASKAWA_UA_DRY_RUN", default=False)
    probe = sdk_probe()
    return {
        "provider": "UnderAutomation.Yaskawa",
        "protocol": "HSES (UDP 10040/10041) + optional FTP/HTTP",
        "product_url": "https://underautomation.com/yaskawa",
        "github": [
            "https://github.com/underautomation/Yaskawa.py",
            "https://github.com/underautomation/Yaskawa.NET",
        ],
        "role": (
            "Live upload / start of generated .JBI jobs and controller status. "
            "Offline seam planning and INFORM postprocessing stay in this app."
        ),
        "host_configured": bool(host),
        "host": host or None,
        "dry_run": dry,
        "sdk": probe,
        "license_registered": bool(
            os.environ.get("YASKAWA_UA_LICENSE_COMPANY")
            and os.environ.get("YASKAWA_UA_LICENSE_KEY")
        ),
        "controllers": [
            "DX200",
            "DX100",
            "YRC1000",
            "YRC1000micro",
            "FS100",
            "MOTOMAN NEXT",
        ],
    }


def _register_license() -> None:
    company = os.environ.get("YASKAWA_UA_LICENSE_COMPANY", "").strip()
    key = os.environ.get("YASKAWA_UA_LICENSE_KEY", "").strip()
    if not company or not key:
        return
    from underautomation.yaskawa.yaskawa_robot import YaskawaRobot

    YaskawaRobot.register_license(company, key)


def _require_host() -> str:
    host = os.environ.get("YASKAWA_ROBOT_HOST", "").strip()
    if not host:
        raise RuntimeError(
            "Set YASKAWA_ROBOT_HOST to the DX200 / Motoman controller IP "
            "(or YASKAWA_UA_DRY_RUN=1 for a simulated response)."
        )
    return host


def _connect():
    from underautomation.yaskawa.connect_parameters import ConnectParameters
    from underautomation.yaskawa.yaskawa_robot import YaskawaRobot

    _register_license()
    host = _require_host()
    robot = YaskawaRobot()
    params = ConnectParameters(host)
    # FTP needed for reliable .JBI push on many DX200 cells.
    try:
        params.ftp.enable = True
    except Exception:  # noqa: BLE001
        pass
    robot.connect(params)
    return robot


def get_status() -> dict[str, Any]:
    if _env_bool("YASKAWA_UA_DRY_RUN"):
        return {
            "dry_run": True,
            "servo_on": False,
            "alarming": False,
            "running": False,
            "teach": True,
            "play": False,
            "mode": "simulated",
        }
    probe = sdk_probe()
    if not probe["available"]:
        raise RuntimeError(probe.get("error") or probe.get("hint") or "SDK unavailable")

    robot = _connect()
    try:
        st = robot.high_speed_e_server.get_status_information()
        return {
            "dry_run": False,
            "servo_on": bool(getattr(st, "servo_on", False)),
            "alarming": bool(getattr(st, "alarming", False)),
            "running": bool(getattr(st, "running", False)),
            "teach": bool(getattr(st, "teach", False)),
            "play": bool(getattr(st, "play", False)),
            "raw_type": type(st).__name__,
        }
    finally:
        try:
            robot.disconnect()
        except Exception:  # noqa: BLE001
            pass


def upload_jbi(jbi_text: str, job_name: str | None = None) -> dict[str, Any]:
    """Upload INFORM job text to the controller (FTP preferred)."""
    text = jbi_text if jbi_text.endswith("\n") else jbi_text + "\n"
    name = (job_name or "CADCAM").strip()
    if name.upper().endswith(".JBI"):
        name = name[:-4]
    name = name[:32] or "CADCAM"
    remote = f"{name}.JBI"

    if _env_bool("YASKAWA_UA_DRY_RUN"):
        return {
            "dry_run": True,
            "job": name,
            "remote": remote,
            "bytes": len(text.encode("utf-8", errors="replace")),
            "ok": True,
        }

    probe = sdk_probe()
    if not probe["available"]:
        raise RuntimeError(probe.get("error") or probe.get("hint") or "SDK unavailable")

    robot = _connect()
    try:
        data = text.encode("utf-8", errors="replace")
        uploaded_via = "ftp"
        try:
            # Prefer FTP put when enabled.
            tmp = Path(tempfile.gettempdir()) / remote
            tmp.write_bytes(data)
            ftp = getattr(robot, "ftp", None)
            if ftp is None:
                raise RuntimeError("FTP client not available on YaskawaRobot")
            # API variants across SDK versions.
            if hasattr(ftp, "upload_file"):
                ftp.upload_file(str(tmp), remote)
            elif hasattr(ftp, "UploadFile"):
                ftp.UploadFile(str(tmp), remote)
            else:
                raise RuntimeError("No FTP upload method on SDK client")
            try:
                tmp.unlink(missing_ok=True)
            except Exception:  # noqa: BLE001
                pass
        except Exception as ftp_exc:  # noqa: BLE001
            # Fallback: HighSpeedEServer file write if present.
            hses = robot.high_speed_e_server
            if hasattr(hses, "write_file"):
                hses.write_file(remote, data)
                uploaded_via = "hses_write_file"
            else:
                raise RuntimeError(
                    f"FTP upload failed ({ftp_exc}); HSES write_file unavailable"
                ) from ftp_exc

        return {
            "dry_run": False,
            "job": name,
            "remote": remote,
            "bytes": len(data),
            "via": uploaded_via,
            "ok": True,
        }
    finally:
        try:
            robot.disconnect()
        except Exception:  # noqa: BLE001
            pass


def select_and_start_job(job_name: str, line: int = 0, servo_on: bool = True) -> dict[str, Any]:
    """Select a job on the controller and start playback (remote + play required)."""
    name = job_name.strip()
    if name.upper().endswith(".JBI"):
        name = name[:-4]
    if not name:
        raise ValueError("job_name required")

    if _env_bool("YASKAWA_UA_DRY_RUN"):
        return {
            "dry_run": True,
            "job": name,
            "line": line,
            "servo_on": servo_on,
            "started": True,
        }

    probe = sdk_probe()
    if not probe["available"]:
        raise RuntimeError(probe.get("error") or probe.get("hint") or "SDK unavailable")

    robot = _connect()
    try:
        hses = robot.high_speed_e_server
        hses.select_job(name, int(line))
        if servo_on and hasattr(hses, "set_servo"):
            hses.set_servo(True)
        hses.start_job()
        return {
            "dry_run": False,
            "job": name,
            "line": int(line),
            "servo_on": servo_on,
            "started": True,
        }
    finally:
        try:
            robot.disconnect()
        except Exception:  # noqa: BLE001
            pass


def upload_and_start(jbi_text: str, job_name: str | None = None, line: int = 0) -> dict[str, Any]:
    """Convenience: upload generated CAD/CAM job then start it."""
    up = upload_jbi(jbi_text, job_name=job_name)
    start = select_and_start_job(up["job"], line=line, servo_on=True)
    return {"upload": up, "start": start}
