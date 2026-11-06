"""
Welding power-source configuration — Motoweld-E on DX200 ARC WELDING.

Production dump (`reference/dx200-backup-20260915` / `robot-cell-dx200`):
  * ``WELDER.DAT`` → MOTOWELD-E Series 350A
  * Jobs: ``MACRO1 MJ#(0) ARGFn`` then bare ``ARCON`` / ``ARCOF`` (no ``ASF#``)
  * Conditions live in ``ARCSRT.CND`` / ``WELDUDEF.DAT``; weave in ``WEAV.CND``
  * ``MACRO.DAT`` MACRO1#1 is still labelled ``LORCH`` (legacy macro name) —
    ``ARGF`` is the PROG-NR passed into that macro.

============================================================================
  !!!  UWAGA / WARNING — UNQUALIFIED WPS  !!!
============================================================================
UI amps/volts are APP_ESTIMATE heuristics for offline review only.
They are NOT a qualified WPS. Verify MACRO1 ARGF + WEAV on the pendant /
MotoSim before any DX200 load.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WeldSchedule:
    """One reusable welding condition shown in the UI / JBI comments."""

    condition_no: int
    argf: int  # MACRO1 MJ#(0) ARGF<n> as used on the cell
    current_a: float
    voltage_v: float
    wire_speed: float
    travel_speed: float
    gas: str = "CO2/Mix (cell)"
    process: str = "Motoweld"
    weldudef_user: int | None = None
    arcsrt_en: int | None = None
    source_jobs: tuple[str, ...] = ()
    notes: str = ""

    @property
    def lorch_job(self) -> int:
        """Backward-compat alias — same integer as ``argf``."""
        return self.argf


# Production ARGF values seen in verified hall jobs (M3.1).
# ARCSRT EN 1/4/5/8 in this dump share identical template rows
# (line ``200,20.0,100``) — do NOT treat UI A/V as pendant-certified.
MOTOWELD_SCHEDULES: dict[int, WeldSchedule] = {
    1: WeldSchedule(
        condition_no=1,
        argf=1,
        current_a=180.0,
        voltage_v=22.0,
        wire_speed=6.5,
        travel_speed=5.0,
        weldudef_user=1,
        arcsrt_en=1,
        source_jobs=("11.JBI", "12.JBI", "123.JBI"),
        notes="MACRO1 ARGF1 — short ST1/ST2 jobs; WELDUDEF USER-1",
    ),
    4: WeldSchedule(
        condition_no=4,
        argf=4,
        current_a=200.0,
        voltage_v=20.0,
        wire_speed=7.0,
        travel_speed=5.0,
        weldudef_user=4,
        arcsrt_en=4,
        source_jobs=("1-DEFRO-5.JBI", "02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI"),
        notes="MACRO1 ARGF4 — DEFRO / KOSZ; WELDUDEF USER-4",
    ),
    5: WeldSchedule(
        condition_no=5,
        argf=5,
        current_a=180.0,
        voltage_v=22.0,
        wire_speed=6.5,
        travel_speed=5.0,
        weldudef_user=5,
        arcsrt_en=5,
        source_jobs=("1-DEFRO-5-KOSZ.JBI",),
        notes="MACRO1 ARGF5 — 1-DEFRO-5-KOSZ; WELDUDEF USER-5",
    ),
    8: WeldSchedule(
        condition_no=8,
        argf=8,
        current_a=220.0,
        voltage_v=24.0,
        wire_speed=8.0,
        travel_speed=6.7,
        weldudef_user=8,
        arcsrt_en=8,
        source_jobs=("000.JBI",),
        notes="MACRO1 ARGF8 — 000.JBI park/demo weld; WELDUDEF USER-8",
    ),
}

# Legacy alias (tests / older imports). Prefer MOTOWELD_SCHEDULES.
LORCH_S8_SCHEDULES = MOTOWELD_SCHEDULES


@dataclass(frozen=True)
class WeaveOption:
    """Live-cell WEV# choice (None = no WVON)."""

    wev: int | None  # None → omit WVON/WVOF
    label: str
    source_jobs: tuple[str, ...] = ()
    weav_cnd_summary: str = ""


# M3.3 — dropdown set requested for UI (plus dump notes in CELL_WELD_MAP.md).
CELL_WEAVE_OPTIONS: tuple[WeaveOption, ...] = (
    WeaveOption(None, "Brak / None", (), "no WVON"),
    WeaveOption(1, "WEV#(1)", ("1-DEFRO-5-KOSZ.JBI",), "amp 2/2 mm, freq-ish 70/70"),
    WeaveOption(15, "WEV#(15)", ("1KT.JBI", "02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI"), "3/3 mm, 45°"),
    WeaveOption(16, "WEV#(16)", ("02025_KOSZ_DO_RURY_70-PLUS-RAMKI.JBI",), "2/2 mm, 45°"),
    WeaveOption(21, "WEV#(21)", ("-DEFRO-70-PRZYLACZE-70-KOPIA-.JBI",), "4 mm amp; WEAV.CND slot 21"),
)

DEFAULT_WEAVE_NO: int | None = 21
DEFAULT_ARGF: int = 1

WPS_UNQUALIFIED_WARNING = (
    "UNQUALIFIED WPS — VERIFY MACRO1 ARGF AND WEAV ON PENDANT"
)


def weave_options_public() -> list[dict]:
    return [
        {
            "wev": w.wev,
            "label": w.label,
            "source_jobs": list(w.source_jobs),
            "weav_cnd_summary": w.weav_cnd_summary,
        }
        for w in CELL_WEAVE_OPTIONS
    ]


def schedules_public() -> list[dict]:
    out: list[dict] = []
    for s in MOTOWELD_SCHEDULES.values():
        out.append(
            {
                "condition_no": s.condition_no,
                "argf": s.argf,
                "lorch_job": s.lorch_job,  # alias
                "current_a": s.current_a,
                "voltage_v": s.voltage_v,
                "wire_speed": s.wire_speed,
                "travel_speed": s.travel_speed,
                "process": s.process,
                "gas": s.gas,
                "weldudef_user": s.weldudef_user,
                "arcsrt_en": s.arcsrt_en,
                "source_jobs": list(s.source_jobs),
                "notes": s.notes,
                "wps_status": "UNQUALIFIED_HEURISTIC",
            }
        )
    return out


@dataclass
class PowerSourceConfig:
    name: str = "MOTOWELD-E Series 350A"
    interface: str = "arc-interface"
    default_condition: int = 1
    cell_label: str = "MA2010 · DX200 · RECT-X · TURN"


ACTIVE_POWER_SOURCE = PowerSourceConfig()


def get_schedule(condition_no: int) -> WeldSchedule:
    if condition_no not in MOTOWELD_SCHEDULES:
        raise KeyError(
            f"Weld condition {condition_no} not defined. "
            f"Available: {sorted(MOTOWELD_SCHEDULES)}"
        )
    return MOTOWELD_SCHEDULES[condition_no]


def get_schedule_by_argf(argf: int) -> WeldSchedule:
    for s in MOTOWELD_SCHEDULES.values():
        if s.argf == argf:
            return s
    raise KeyError(f"No Motoweld schedule for ARGF{argf}")
