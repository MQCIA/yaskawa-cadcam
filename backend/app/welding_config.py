"""
Welding power-source configuration: Lorch S8 (SpeedPulse XT class).

Maps logical weld schedules to the values referenced by DX200 ARCON/ARCOF
instructions. On a real cell the robot->power-source link is usually a digital
fieldbus (EtherNet/IP, DeviceNet or PROFINET) or the analog/ARC interface; the
DX200 ARCON typically references a weld condition / ASF file number, while the
actual amps/volts/synergic-line live either in that condition file or are sent
over the bus.

============================================================================
  !!!  UWAGA / WARNING  !!!
============================================================================
The numbers below are EXAMPLE weld parameters, not a qualified WPS. Real
current/voltage/wire-feed/synergic-program values must come from your welding
procedure specification and be validated on coupons. The mapping between an
ARCON condition number and a Lorch S8 job/program depends entirely on how your
integrator wired and configured the interface. Confirm every value.
============================================================================
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WeldSchedule:
    """One reusable welding condition.

    condition_no : number referenced by ARCON on the DX200 (ASF# / weld file)
    lorch_job    : Lorch S8 job/program number selected on the power source
    current_a    : target welding current [A]              (EXAMPLE value)
    voltage_v    : target arc voltage [V]                  (EXAMPLE value)
    wire_speed   : wire feed speed [m/min]                 (EXAMPLE value)
    travel_speed : recommended travel/weld speed [cm/min]  (EXAMPLE value)
    gas          : shielding gas description
    """

    condition_no: int
    lorch_job: int
    current_a: float
    voltage_v: float
    wire_speed: float
    travel_speed: float
    gas: str = "M21 (82% Ar / 18% CO2)"
    process: str = "SpeedPulse"


# Example library. Replace with your qualified WPS values.
LORCH_S8_SCHEDULES: dict[int, WeldSchedule] = {
    1: WeldSchedule(1, lorch_job=1, current_a=180, voltage_v=22.0, wire_speed=6.5,
                    travel_speed=40, process="SpeedPulse"),
    2: WeldSchedule(2, lorch_job=2, current_a=240, voltage_v=26.0, wire_speed=9.0,
                    travel_speed=35, process="SpeedPulse"),
    3: WeldSchedule(3, lorch_job=3, current_a=120, voltage_v=18.5, wire_speed=4.0,
                    travel_speed=45, process="SpeedArc"),
}


@dataclass
class PowerSourceConfig:
    name: str = "Lorch S8"
    interface: str = "digital"  # "digital" (fieldbus) | "analog" | "arc-interface"
    default_condition: int = 1


ACTIVE_POWER_SOURCE = PowerSourceConfig()


def get_schedule(condition_no: int) -> WeldSchedule:
    if condition_no not in LORCH_S8_SCHEDULES:
        raise KeyError(
            f"Weld condition {condition_no} not defined. "
            f"Available: {sorted(LORCH_S8_SCHEDULES)}"
        )
    return LORCH_S8_SCHEDULES[condition_no]
