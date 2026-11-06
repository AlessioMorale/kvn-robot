# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""The calibration file /etc/kvn/calibration_<hostname>.yaml, as ROS 2 parameters.

  /**:
    ros__parameters:
      extrinsics:
        base_link_to_imu:   {translation: [x, y, z], rotation_rpy: [roll, pitch, yaw]}
        base_link_to_oakd:  {...}
        base_link_to_lidar: {...}
  skid_steer_controller:          # wheel odometry calibration, when it was run
    ros__parameters: {wheel_radius: ..., track_width: ..., chi: ...}
  odometry_node:
    ros__parameters: {...}

Saving merges into the existing file as nested dicts, so a re-run overwrites the values it
computed, keeps the ones it skipped, and can never produce duplicate keys.
"""
import copy
from datetime import datetime
import os
import socket

import yaml

WILDCARD = '/**'
SENSORS = ('imu', 'oakd', 'lidar')


def default_path(directory='/etc/kvn'):
    return os.path.join(directory, f'calibration_{socket.gethostname()}.yaml')


def deep_merge(base, update):
    """Copy of base with update merged in: dicts recursively, everything else replaced."""
    merged = copy.deepcopy(base) if isinstance(base, dict) else {}
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def load(path):
    """Existing calibration as a dict; {} when there is none. ValueError when it does not parse."""
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except FileNotFoundError:
        return {}
    except yaml.YAMLError as e:
        raise ValueError(f'{path} is not valid YAML: {e}') from e
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f'{path} does not hold a parameter mapping')
    return data


def extrinsics(document):
    """{sensor: {'translation': [...], 'rotation_rpy': [...]}} of a calibration document."""
    try:
        section = document[WILDCARD]['ros__parameters']['extrinsics']
    except (KeyError, TypeError):
        return {}
    return {name[len('base_link_to_'):]: value for name, value in section.items()
            if name.startswith('base_link_to_') and isinstance(value, dict)}


def _floats(values):
    return [round(float(v), 6) for v in values]


def build_update(sensors, odometry=None, odometry_nodes=()):
    """Document fragment for the computed values.

    sensors: {sensor: {'translation': xyz, 'rotation_rpy': rpy}} (either key optional).
    odometry: {'wheel_radius', 'track_width', 'chi'} written for each node in odometry_nodes.
    """
    section = {}
    for sensor, entry in sensors.items():
        section[f'base_link_to_{sensor}'] = {
            key: _floats(entry[key]) for key in ('translation', 'rotation_rpy') if key in entry}
    update = {WILDCARD: {'ros__parameters': {'extrinsics': section}}} if section else {}
    if odometry:
        values = {k: round(float(v), 6) for k, v in odometry.items()}
        for node in odometry_nodes:
            update[node.lstrip('/').split('/')[-1]] = {'ros__parameters': dict(values)}
    return update


def render(document, stamp=None):
    stamp = stamp or datetime.now().isoformat(timespec='seconds')
    header = (f'# KVN calibration of {socket.gethostname()}, written by kvn_calibration_supervisor '
              f'on {stamp}.\n# Re-running the calibration rewrites this file; edits are kept only '
              'for the values a run skips.\n')
    # Wildcard section first, node sections after, as in the docstring
    ordered = {k: document[k] for k in sorted(document, key=lambda k: (k != WILDCARD, k))}
    return header + yaml.safe_dump(ordered, sort_keys=False, default_flow_style=None)


def save(path, update, stamp=None):
    """Merges update into the file at path and rewrites it. Returns the merged document."""
    document = deep_merge(load(path), update)
    text = render(document, stamp)  # serialise first: a failure must not truncate the file
    directory = os.path.dirname(path) or '.'
    tmp = os.path.join(directory, f'.{os.path.basename(path)}.tmp')
    try:
        with open(tmp, 'w', encoding='utf-8') as stream:
            stream.write(text)
        os.replace(tmp, path)
    except PermissionError:
        # /etc/kvn is root's; install.sh only hands the file itself to the service user
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write(text)
    return document
