"""
Welding power-source configuration — Motoweld-E on the production DX200 cell.

Mined from CF dump fixtures (`backend/tests/fixtures/robot-cell/`, same files as
`reference/robot-cell-dx200`):

  * ``MACRO.DAT`` ``//MACRO1 1`` → ``///JOB1 LORCH`` / tag ``PROG-NR:``
    so ``MACRO1 MJ#(0) ARGFn`` selects Motoweld program **n** (not an ASF#).
  * ``ARCSRT.CND`` — all 1000 ``//ARCSRT EN`` blocks are **identical** templates;
    the teachable arc-start row used on this dump is ``200,20.0,100``
    (200 A / 20.0 V / 100 %). Wire-related field ``13.3`` appears on the
    comment/aux lines; Motoweld synergic curves live in ``WELDER.DAT`` /
    ``WELDUDEF.DAT``.
  * ``WEAV.CND`` — ``WEV#(1|15|16|21)`` amplitudes / angles / frequency.
  * ``123123.JBI`` — TouchSense search uses ``MOVS … V=11.0 +MOVJ EC# VJ=0.78``.

See ``docs/CELL_WELD_MAP.md`` and ``docs/CELL.md``.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
A/V/wire below combine the shared ARCSRT template with Motoweld PROG-NR (=ARGF)
and travel speeds observed next to those ARGFs in live jobs. They are **not**
a qualified WPS — validate on the Motoweld pendant and in MotoSim.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Shared ARCSRT.CND template (EN 1 … EN 1000 identical on this dump)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ArcSrtTemplate:
    """Decoded fields from one ``//ARCSRT EN`` block."""

    en: int
    current_a: float
    voltage_v: float
    percent: float
    wire_hint: float  # aux field seen as 13.3 next to condition lines


ARCSRT_TEMPLATE = ArcSrtTemplate(
    en=1,
    current_a=200.0,
    voltage_v=20.0,
    percent=100.0,
    wire_hint=13.3,
)

# Motoweld factory curve sample (WELDER.DAT) — wire m/min ↔ A
MOTOWELD_WIRE_M_MIN = (0.00, 1.35, 2.70, 10.80, 12.15, 13.50)
MOTOWELD_CURRENT_A = (30, 62, 94, 286, 318, 350)


def wire_for_current(amps: float) -> float:
    """Linear interpolate Motoweld ``WELDER.DAT`` wire feed for a given current."""
    xs = MOTOWELD_CURRENT_A
    ys = MOTOWELD_WIRE_M_MIN
    if amps <= xs[0]:
        return ys[0]
    if amps >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= amps <= xs[i + 1]:
            t = (amps - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]


# ---------------------------------------------------------------------------
# Motoweld ARGF (= PROG-NR) schedules used in production jobs
# ---------------------------------------------------------------------------

@dataclass
class WeldSchedule:
    """One reusable welding condition shown in the UI / JBI comments."""

    condition_no: int
    lorch_job: int  # Motoweld PROG-NR = ARGF value in MACRO1 MJ#(0) ARGFn
    current_a: float
    voltage_v: float
    wire_speed: float
    travel_speed: float
    gas: str = "CO2/Mix (cell)"
    process: str = "Motoweld"
    default_weave: int = 21
    arcsrt_en: int = 1  # all EN identical; documented for map completeness
    note: str = ""


_ARCSRT_A = ARCSRT_TEMPLATE.current_a
_ARCSRT_V = ARCSRT_TEMPLATE.voltage_v
_WIRE_200 = round(wire_for_current(_ARCSRT_A), 1)  # ≈ 7.2 m/min


# Keys = UI condition numbers. ``lorch_job`` = real ARGF / Motoweld PROG-NR.
# Travel speeds mined from jobs that call the matching ARGF (see CELL_WELD_MAP).
LORCH_S8_SCHEDULES: dict[int, WeldSchedule] = {
    1: WeldSchedule(
        1,
        lorch_job=8,
        current_a=_ARCSRT_A,
        voltage_v=_ARCSRT_V,
        wire_speed=_WIRE_200,
        travel_speed=5.0,
        default_weave=21,
        note="Default production (000.JBI / DEFRO) — ARGF8 + WEV#(21)",
    ),
    2: WeldSchedule(
        2,
        lorch_job=5,
        current_a=_ARCSRT_A,
        voltage_v=_ARCSRT_V,
        wire_speed=_WIRE_200,
        travel_speed=4.2,
        default_weave=1,
        note="ARGF5 — 1.JBI / 1C / 1-DEFRO-5-KOSZ, V≈4.2",
    ),
    3: WeldSchedule(
        3,
        lorch_job=4,
        current_a=_ARCSRT_A,
        voltage_v=_ARCSRT_V,
        wire_speed=_WIRE_200,
        travel_speed=5.0,
        default_weave=15,
        note="ARGF4 — DEFRO / kosz; often WEV#(15)/WEV#(16)",
    ),
    4: WeldSchedule(
        4,
        lorch_job=1,
        current_a=_ARCSRT_A,
        voltage_v=_ARCSRT_V,
        wire_speed=_WIRE_200,
        travel_speed=5.0,
        default_weave=1,
        note="ARGF1 — 12.JBI / 11.JBI / 123.JBI",
    ),
}

# Direct ARGF → schedule lookup (Motoweld PROG-NR)
MOTOWELD_ARGF_SCHEDULES: dict[int, WeldSchedule] = {
    s.lorch_job: s for s in LORCH_S8_SCHEDULES.values()
}

# Alias for clarity in new code
MOTOWELD_SCHEDULES = LORCH_S8_SCHEDULES


# ---------------------------------------------------------------------------
# Weave files WEV#(n) from WEAV.CND
# Row format after header: amp_l, amp_r, ang_l, ang_r, freq, travel_or_mode
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class WeaveFile:
    wev_no: int
    mode: int
    amp_l_mm: float
    amp_r_mm: float
    angle_l_deg: float
    angle_r_deg: float
    frequency_hz: float
    aux: float
    used_in_jobs: str


# Parsed from WEAV.CND for numbers used in live jobs (user request: 1, 15, 16, 21).
WEAVE_FILES: dict[int, WeaveFile] = {
    1: WeaveFile(
        1, 0, 2.0, 2.0, 70.0, 70.0, 2.0, 2.0,
        "DEFRO-60_STARE, 1-DEFRO-5-KOSZ, KASUJ",
    ),
    15: WeaveFile(
        15, 1, 3.0, 3.0, 45.0, 45.0, 3.0, 5.0,
        "1KT.JBI, 02025_KOSZ… (+ARGF4)",
    ),
    16: WeaveFile(
        16, 1, 2.0, 2.0, 45.0, 0.0, 2.0, 1.0,
        "02025_KOSZ… after WEV#(15)",
    ),
    21: WeaveFile(
        21, 0, 4.0, 0.0, 0.0, 0.0, 2.0, 3.5,
        "DEFRO-70-NOWE/KOPIA default production weave",
    ),
}

DEFAULT_WEAVE_NO = 21


# ---------------------------------------------------------------------------
# TouchSense / search (123123.JBI)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TouchSenseConfig:
    """INFORM search move matching ``123123.JBI``."""

    opcode: str = "MOVS"
    search_speed_cm_min: float = 11.0  # V=11.0 on MOVS
    station_vj: float = 0.78  # +MOVJ EC# VJ=0.78
    reference_job: str = "123123.JBI"


TOUCH_SENSE = TouchSenseConfig()


@dataclass
class PowerSourceConfig:
    name: str = "MOTOWELD-E Series 350A"
    interface: str = "arc-interface"
    default_condition: int = 1
    default_argf: int = 8
    default_weave: int = DEFAULT_WEAVE_NO


ACTIVE_POWER_SOURCE = PowerSourceConfig()


def get_schedule(condition_no: int) -> WeldSchedule:
    if condition_no not in LORCH_S8_SCHEDULES:
        raise KeyError(
            f"Weld condition {condition_no} not defined. "
            f"Available: {sorted(LORCH_S8_SCHEDULES)}"
        )
    return LORCH_S8_SCHEDULES[condition_no]


def get_schedule_by_argf(argf: int) -> WeldSchedule:
    if argf not in MOTOWELD_ARGF_SCHEDULES:
        raise KeyError(
            f"Motoweld ARGF{argf} not in mined map. "
            f"Available: {sorted(MOTOWELD_ARGF_SCHEDULES)}"
        )
    return MOTOWELD_ARGF_SCHEDULES[argf]


def get_weave(wev_no: int) -> WeaveFile:
    if wev_no not in WEAVE_FILES:
        raise KeyError(
            f"WEV#({wev_no}) not in mined map. Available: {sorted(WEAVE_FILES)}"
        )
    return WEAVE_FILES[wev_no]
