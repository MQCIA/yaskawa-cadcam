"""
Welding power-source configuration.

Production cell dump (`reference/robot-cell-dx200`, SYSTEM.SYS / WELDER.DAT):
  MOTOWELD-E Series 350A class on DX200 ARC WELDING application.

Jobs on that cell call plain ``ARCON`` / ``ARCOF`` (no ``ASF#(...)``); weld
parameters live in ``ARCSRT.CND`` / ``WELDUDEF.DAT`` and are selected via teach
macros (``MACRO1 MJ#(0) ARGFn``) rather than Lorch job numbers.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
Schedule numbers below are EXAMPLE values for the CAD/CAM UI. Real amps/volts
must come from your WPS and the conditions stored on the controller.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WeldSchedule:
    """One reusable welding condition shown in the UI / JBI comments."""

    condition_no: int
    lorch_job: int  # legacy field name; for Motoweld = user condition / ARGF hint
    current_a: float
    voltage_v: float
    wire_speed: float
    travel_speed: float
    gas: str = "M21 (82% Ar / 18% CO2)"
    process: str = "MIG/MAG"


# UI library — travel speeds (~5–8) match magnitudes seen in cell .JBI (V=5.0…).
LORCH_S8_SCHEDULES: dict[int, WeldSchedule] = {
    1: WeldSchedule(1, lorch_job=8, current_a=180, voltage_v=22.0, wire_speed=6.5,
                    travel_speed=5.0, process="Motoweld", gas="CO2/Mix (cell)"),
    2: WeldSchedule(2, lorch_job=8, current_a=220, voltage_v=24.0, wire_speed=8.0,
                    travel_speed=6.7, process="Motoweld", gas="CO2/Mix (cell)"),
    3: WeldSchedule(3, lorch_job=4, current_a=140, voltage_v=19.0, wire_speed=5.0,
                    travel_speed=7.5, process="Motoweld", gas="CO2/Mix (cell)"),
}

# Alias for clarity in new code
MOTOWELD_SCHEDULES = LORCH_S8_SCHEDULES


@dataclass
class PowerSourceConfig:
    name: str = "MOTOWELD-E Series 350A"
    interface: str = "arc-interface"
    default_condition: int = 1


ACTIVE_POWER_SOURCE = PowerSourceConfig()


def get_schedule(condition_no: int) -> WeldSchedule:
    if condition_no not in LORCH_S8_SCHEDULES:
        raise KeyError(
            f"Weld condition {condition_no} not defined. "
            f"Available: {sorted(LORCH_S8_SCHEDULES)}"
        )
    return LORCH_S8_SCHEDULES[condition_no]
