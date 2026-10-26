# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

from kvn_wled_signal import select_mode


def test_startup_and_standby():
    assert select_mode(None, None, False, False, False, 13.6, 12.4) == "starting_up"
    assert select_mode("RDY", None, False, False, False, 13.6, 12.4) == "standby"


def test_robot_activity_priority():
    assert select_mode("RDY", 15.0, True, False, False, 13.6, 12.4) == "teleop"
    assert select_mode("RDY", 15.0, True, False, True, 13.6, 12.4) == "navigation"
    assert select_mode("RDY", 15.0, True, True, True, 13.6, 12.4) == "exploration"


def test_battery_modes_override_activity():
    assert select_mode("DRV", 13.6, True, True, True, 13.6, 12.4) == "battery_low"
    assert select_mode("DRV", 12.4, True, True, True, 13.6, 12.4) == "battery_critical"
    assert select_mode("DRV", float("nan"), True, False, False, 13.6, 12.4) == (
        "battery_critical"
    )
    assert select_mode("WRN:BATT", None, False, False, False, 13.6, 12.4) == (
        "battery_low"
    )
    assert select_mode("FLT:BATT", None, False, False, False, 13.6, 12.4) == (
        "battery_critical"
    )


def test_robot_faults_override_activity_and_low_battery():
    assert select_mode("FLT:HWMON", 13.0, True, True, True, 13.6, 12.4) == "fault"
    assert select_mode("WRN:HWMON", 15.0, True, False, False, 13.6, 12.4) == (
        "warning"
    )
