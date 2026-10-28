# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

from kvn_wled_signal import (
    FX_BLINK,
    FX_BREATHE,
    MODE_SETTINGS,
    WLED_FRAME_MS,
    blink_timing,
    resolve_mode,
    select_mode,
)


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


def test_orchestrator_signal_wins_when_known():
    assert resolve_mode("assisted_teleop", "standby") == "assisted_teleop"
    assert resolve_mode("returning_home", "teleop") == "returning_home"


def test_local_mode_without_a_usable_signal():
    assert resolve_mode(None, "battery_low") == "battery_low"
    assert resolve_mode("no_such_pattern", "standby") == "standby"


def test_every_orchestrator_signal_has_a_pattern():
    signals = {
        "battery_critical", "fault", "battery_low", "warning", "returning_home",
        "exploration", "navigation", "assisted_teleop", "teleop", "starting_up", "standby",
    }
    assert signals <= set(MODE_SETTINGS)


def test_blink_timing_matches_wled_blink():
    # cycle = (255 - sx) * 20 ms + 2 frames; on = 1 frame + (cycle - 2 frames) * ix / 256
    for period_s in (2.0, 1.0, 0.5, 0.25):
        speed, intensity = blink_timing(period_s)
        cycle_ms = (255 - speed) * 20
        assert abs(cycle_ms + 2 * WLED_FRAME_MS - period_s * 1000) <= 10
        on_ms = WLED_FRAME_MS + cycle_ms * intensity / 256
        assert abs(on_ms - 0.1 * period_s * 1000) <= 5


def test_mode_patterns():
    assert MODE_SETTINGS["starting_up"][:2] == ((255, 255, 0), FX_BREATHE)
    for mode, period_s in {
        "standby": 2.0, "teleop": 1.0, "assisted_teleop": 1.0, "navigation": 1.0,
        "exploration": 1.0, "battery_low": 0.5, "battery_critical": 0.5, "warning": 0.25,
        "fault": 0.25,
    }.items():
        assert MODE_SETTINGS[mode][1:] == (FX_BLINK, *blink_timing(period_s)), mode
