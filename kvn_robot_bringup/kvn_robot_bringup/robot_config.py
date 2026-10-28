# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Robot-wide hardware configuration (config/robot.yaml) for the launch files.

robot.yaml holds the values that describe this particular rover's hardware: device ports and
battery limits. Several nodes need the same values under different parameter names (the battery
thresholds are read by hwmon, kvn_status, the WLED node and the orchestrator), so the launch
files read robot.yaml once and hand each node its share:

    cfg = robot_config.load()
    Node(..., parameters=[node_yaml, robot_config.node_overrides(cfg)['crsf_joy_node']])
    # or, for a launch file of another package that takes a params file argument:
    merged = robot_config.merged_params_file(node_yaml, 'kvn_status_node', overrides)

The file is taken from $KVN_ROBOT_CONFIG when set, else from this package's config/robot.yaml.
"""

import math
import os
import tempfile
from typing import Any

import yaml

ENV_VAR = "KVN_ROBOT_CONFIG"


def default_path() -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("kvn_robot_bringup"), "config", "robot.yaml")


def config_path() -> str:
    return os.environ.get(ENV_VAR) or default_path()


def load(path: str | None = None) -> dict[str, Any]:
    """Reads and validates robot.yaml; raises ValueError naming the offending key."""
    path = path or config_path()
    with open(path, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}
    validate(cfg, path)
    return cfg


def _require(cfg: dict[str, Any], dotted: str, path: str) -> Any:
    value: Any = cfg
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f"{path}: missing '{dotted}'")
        value = value[key]
    return value


def _positive(cfg: dict[str, Any], dotted: str, path: str) -> float:
    value = _require(cfg, dotted, path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path}: '{dotted}' must be a number, got {value!r}")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{path}: '{dotted}' must be finite and > 0, got {value!r}")
    return float(value)


def validate(cfg: dict[str, Any], path: str = "robot.yaml") -> None:
    for port in ("motors", "rc_receiver", "lidar", "imu", "wled"):
        value = _require(cfg, f"ports.{port}", path)
        if not isinstance(value, str) or not value:
            raise ValueError(f"{path}: 'ports.{port}' must be a device path")
    for baud in ("rc_receiver_baudrate", "lidar_baudrate"):
        value = _require(cfg, f"ports.{baud}", path)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{path}: 'ports.{baud}' must be an integer > 0")
    if not isinstance(_require(cfg, "power_monitor", path), str):
        raise ValueError(f"{path}: 'power_monitor' must be a sysfs path")

    cells = _require(cfg, "battery.cells", path)
    if isinstance(cells, bool) or not isinstance(cells, int) or cells <= 0:
        raise ValueError(f"{path}: 'battery.cells' must be an integer > 0")
    critical = _positive(cfg, "battery.critical_voltage", path)
    warn = _positive(cfg, "battery.warn_voltage", path)
    over_warn = _positive(cfg, "battery.overvoltage_warn", path)
    over_error = _positive(cfg, "battery.overvoltage_error", path)
    if not critical < warn < over_warn <= over_error:
        raise ValueError(
            f"{path}: battery voltages must satisfy critical_voltage < warn_voltage < "
            "overvoltage_warn <= overvoltage_error"
        )
    _positive(cfg, "battery.capacity_wh", path)
    curve = _require(cfg, "battery.cell_voltage_curve", path)
    if (
        not isinstance(curve, list)
        or len(curve) < 2
        or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in curve)
        or any(b < a for a, b in zip(curve, curve[1:]))
    ):
        raise ValueError(
            f"{path}: 'battery.cell_voltage_curve' must list at least 2 increasing voltages"
        )
    _positive(cfg, "return_home.wh_per_m", path)
    if _positive(cfg, "return_home.path_factor", path) < 1.0:
        raise ValueError(f"{path}: 'return_home.path_factor' must be >= 1")
    reserve = _require(cfg, "return_home.reserve_wh", path)
    if isinstance(reserve, bool) or not isinstance(reserve, (int, float)) or reserve < 0:
        raise ValueError(f"{path}: 'return_home.reserve_wh' must be a number >= 0")


def node_overrides(cfg: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Parameter overrides per node name, with dotted names for nested parameters."""
    ports = cfg["ports"]
    battery = cfg["battery"]
    rth = cfg["return_home"]
    return {
        "crsf_joy_node": {
            "serial.port": ports["rc_receiver"],
            "serial.baudrate": ports["rc_receiver_baudrate"],
        },
        "ism330dhcx_node": {"spi.device": ports["imu"]},
        "hwmon_diagnostic_updater_node": {
            "power.base_path": cfg["power_monitor"],
            "power.battery_voltage.warn_min": float(battery["warn_voltage"]),
            "power.battery_voltage.error_min": float(battery["critical_voltage"]),
            "power.battery_voltage.warn_max": float(battery["overvoltage_warn"]),
            "power.battery_voltage.error_max": float(battery["overvoltage_error"]),
        },
        # The battery input of kvn_status.yaml is named "battery".
        "kvn_status_node": {
            "battery.warn_voltage": float(battery["warn_voltage"]),
            "battery.fault_voltage": float(battery["critical_voltage"]),
        },
        "kvn_wled_signal_node": {
            "serial_port": ports["wled"],
            "low_battery_voltage": float(battery["warn_voltage"]),
            "critical_battery_voltage": float(battery["critical_voltage"]),
        },
        "kvn_orchestrator_node": {
            "blackboard.battery_warn_voltage": float(battery["warn_voltage"]),
            "blackboard.battery_critical_voltage": float(battery["critical_voltage"]),
            "blackboard.battery_cells": int(battery["cells"]),
            "blackboard.battery_capacity_wh": float(battery["capacity_wh"]),
            "blackboard.battery_cell_voltage_curve": [
                float(v) for v in battery["cell_voltage_curve"]
            ],
            "blackboard.battery_wh_per_m": float(rth["wh_per_m"]),
            "blackboard.battery_path_factor": float(rth["path_factor"]),
            "blackboard.battery_reserve_wh": float(rth["reserve_wh"]),
        },
    }


def lidar_launch_arguments(cfg: dict[str, Any]) -> dict[str, str]:
    """Arguments of rplidar_driver's rplidar_with_args.launch.py."""
    return {
        "serial_port": cfg["ports"]["lidar"],
        "serial_baudrate": str(cfg["ports"]["lidar_baudrate"]),
    }


def xacro_arguments(cfg: dict[str, Any]) -> str:
    """xacro command-line arguments of kvn_rover.urdf.xacro."""
    return f" motor_device:={cfg['ports']['motors']}"


def _set_nested(params: dict[str, Any], dotted: str, value: Any) -> None:
    keys = dotted.split(".")
    for key in keys[:-1]:
        child = params.get(key)
        if not isinstance(child, dict):
            child = params[key] = {}
        params = child
    params[keys[-1]] = value


def merge_params(
    params_yaml: dict[str, Any], node_name: str, overrides: dict[str, Any]
) -> dict[str, Any]:
    """Applies dotted overrides to `node_name: ros__parameters:` of a ROS params file."""
    merged = dict(params_yaml or {})
    section = merged.get(node_name)
    if not isinstance(section, dict):
        section = {}
    section = dict(section)
    ros_parameters = section.get("ros__parameters")
    ros_parameters = dict(ros_parameters) if isinstance(ros_parameters, dict) else {}
    for dotted, value in overrides.items():
        _set_nested(ros_parameters, dotted, value)
    section["ros__parameters"] = ros_parameters
    merged[node_name] = section
    return merged


def merged_params_file(base_file: str, node_name: str, overrides: dict[str, Any]) -> str:
    """Writes base_file with the overrides applied to a temporary file and returns its path.

    For launch files of other packages that take a params file argument (the generated file is
    passed instead of the original one).
    """
    with open(base_file, encoding="utf-8") as stream:
        params_yaml = yaml.safe_load(stream) or {}
    merged = merge_params(params_yaml, node_name, overrides)
    handle, path = tempfile.mkstemp(prefix=f"{node_name}_", suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        yaml.safe_dump(merged, stream, sort_keys=False)
    return path
