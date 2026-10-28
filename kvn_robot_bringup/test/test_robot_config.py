# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

import copy
import os

import pytest
import yaml

from kvn_robot_bringup import robot_config

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")
ROBOT_YAML = os.path.join(CONFIG_DIR, "robot.yaml")


@pytest.fixture
def cfg():
    return robot_config.load(ROBOT_YAML)


def test_shipped_file_is_valid(cfg):
    assert cfg["battery"]["critical_voltage"] < cfg["battery"]["warn_voltage"]


def test_environment_variable_selects_the_file(monkeypatch, tmp_path, cfg):
    custom = copy.deepcopy(cfg)
    custom["ports"]["lidar"] = "/dev/ttyUSB7"
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(custom))
    monkeypatch.setenv(robot_config.ENV_VAR, str(path))
    assert robot_config.load()["ports"]["lidar"] == "/dev/ttyUSB7"


@pytest.mark.parametrize(
    "dotted, value, message",
    [
        ("battery.critical_voltage", 14.0, "critical_voltage < warn_voltage"),
        ("battery.cells", 0, "battery.cells"),
        ("battery.cell_voltage_curve", [4.2, 3.0], "cell_voltage_curve"),
        ("ports.lidar", "", "ports.lidar"),
        ("ports.lidar_baudrate", "fast", "ports.lidar_baudrate"),
        ("return_home.path_factor", 0.5, "path_factor"),
    ],
)
def test_invalid_values_are_rejected(cfg, dotted, value, message):
    section, key = dotted.split(".")
    cfg[section][key] = value
    with pytest.raises(ValueError, match=message):
        robot_config.validate(cfg)


def test_missing_key_is_named(cfg):
    del cfg["battery"]["capacity_wh"]
    with pytest.raises(ValueError, match="battery.capacity_wh"):
        robot_config.validate(cfg)


def test_battery_limits_reach_every_consumer(cfg):
    overrides = robot_config.node_overrides(cfg)
    critical = cfg["battery"]["critical_voltage"]
    assert overrides["hwmon_diagnostic_updater_node"]["power.battery_voltage.error_min"] == critical
    assert overrides["kvn_status_node"]["battery.fault_voltage"] == critical
    assert overrides["kvn_wled_signal_node"]["critical_battery_voltage"] == critical
    assert overrides["kvn_orchestrator_node"]["blackboard.battery_critical_voltage"] == critical


def test_override_names_exist_in_the_node_files(cfg):
    """Every override replaces a parameter the node's own YAML already sets."""
    files = {
        "crsf_joy_node": os.path.join(CONFIG_DIR, "teleop.yaml"),
        "hwmon_diagnostic_updater_node": os.path.join(CONFIG_DIR, "hwmon.yaml"),
        "kvn_status_node": os.path.join(
            CONFIG_DIR, "..", "..", "kvn_status", "config", "kvn_status.yaml"
        ),
        "kvn_wled_signal_node": os.path.join(
            CONFIG_DIR, "..", "..", "kvn_status", "config", "kvn_wled_signal.yaml"
        ),
        "kvn_orchestrator_node": os.path.join(
            CONFIG_DIR, "..", "..", "kvn_orchestrator", "config", "orchestrator.yaml"
        ),
    }
    overrides = robot_config.node_overrides(cfg)
    for node, path in files.items():
        with open(path, encoding="utf-8") as stream:
            params = yaml.safe_load(stream)[node]["ros__parameters"]
        for dotted in overrides[node]:
            value = params
            for key in dotted.split("."):
                assert key in value, f"{node}: '{dotted}' is not in {path}"
                value = value[key]


def test_merged_params_file_keeps_other_values(tmp_path):
    base = tmp_path / "node.yaml"
    base.write_text(
        yaml.safe_dump(
            {"my_node": {"ros__parameters": {"serial": {"port": "/dev/a", "baud": 9600}, "x": 1}}}
        )
    )
    path = robot_config.merged_params_file(str(base), "my_node", {"serial.port": "/dev/b"})
    try:
        with open(path, encoding="utf-8") as stream:
            merged = yaml.safe_load(stream)
    finally:
        os.remove(path)
    assert merged["my_node"]["ros__parameters"] == {
        "serial": {"port": "/dev/b", "baud": 9600},
        "x": 1,
    }


def test_merge_adds_a_missing_node_section():
    merged = robot_config.merge_params({}, "n", {"a.b": 2})
    assert merged == {"n": {"ros__parameters": {"a": {"b": 2}}}}
