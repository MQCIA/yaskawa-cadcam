"""
Welding power-source configuration.

Production cell dump (`reference/robot-cell-dx200`, SYSTEM.SYS / WELDER.DAT):
  **DX200 · MA2010 · RECT-X · TURN-ST1 · MOTOWELD-E 350A**

Jobs on that cell call plain ``ARCON`` / ``ARCOF`` (never ``ARCON ASF#(...)``).
Weld conditions are selected with ``MACRO1 MJ#(0) ARGFn`` before ARCON.
Production jobs commonly use ``ARGF1``, ``ARGF4``, ``ARGF5`` (also 2, 8, 11, 20).

Amps / volts / wire shown in the UI are **placeholder estimates** for the
CAD/CAM prototype. Real numbers live in controller ``ARCSRT.CND`` /
``WELDUDEF.DAT`` and must be verified on the pendant / WPS before production.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
Schedule amps/volts below are EXAMPLE UI values — not certified ARCSRT rows.
Only ``argf`` mirrors the MACRO argument seen in live ``.JBI`` files.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WeldSchedule:
    """One reusable welding condition shown in the UI / JBI comments."""

    condition_no: int
    # MACRO1 MJ#(0) ARGFn — from production jobs (pendant condition selector).
    argf: int
    # UI estimates only — verify against ARCSRT.CND / pendant before use.
    current_a: float
    voltage_v: float
    wire_speed: float
    travel_speed: float
    gas: str = "CO2/Mix (cell)"
    process: str = "MOTOWELD-E"
    # Provenance: which fields are dump-backed vs UI placeholders.
    argf_source: str = "production .JBI MACRO1 ARGF"
    params_source: str = "UI placeholder — verify on pendant / ARCSRT.CND"


# Presets mapped to MACRO arguments used in cell jobs (1-DEFRO / 02025 / 12…).
# Travel V≈5–8 matches magnitudes in production MOVL lines.
MOTOWELD_SCHEDULES: dict[int, WeldSchedule] = {
    1: WeldSchedule(
        1,
        argf=1,
        current_a=180,
        voltage_v=22.0,
        wire_speed=6.5,
        travel_speed=5.0,
    ),
    2: WeldSchedule(
        2,
        argf=4,
        current_a=220,
        voltage_v=24.0,
        wire_speed=8.0,
        travel_speed=6.7,
    ),
    3: WeldSchedule(
        3,
        argf=5,
        current_a=140,
        voltage_v=19.0,
        wire_speed=5.0,
        travel_speed=7.5,
    ),
}

# Weave files seen in production jobs (WEAV.CND has 255 entries).
PRODUCTION_WEAVE_NOS: tuple[int, ...] = (1, 15, 16, 21, 22, 24, 25)


@dataclass
class PowerSourceConfig:
    name: str = "MOTOWELD-E Series 350A"
    interface: str = "arc-interface"
    default_condition: int = 1
    cell_label: str = "DX200 · MA2010 · RECT-X · TURN-ST1 · MOTOWELD-E 350A"


ACTIVE_POWER_SOURCE = PowerSourceConfig()


def get_schedule(condition_no: int) -> WeldSchedule:
    if condition_no not in MOTOWELD_SCHEDULES:
        raise KeyError(
            f"Weld condition {condition_no} not defined. "
            f"Available: {sorted(MOTOWELD_SCHEDULES)}"
        )
    return MOTOWELD_SCHEDULES[condition_no]
