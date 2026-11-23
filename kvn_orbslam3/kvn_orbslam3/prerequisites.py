# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.
"""Build-time check for external ORB-SLAM3 C++ prerequisites."""

from __future__ import annotations

import os
import sys
from typing import TextIO

ENV_PREFIX = 'KVN_ORBSLAM3_PREFIX'
DEFAULT_PREFIX = '/opt/orbslam3'

REQUIRED_FILES = (
    os.path.join('lib', 'libORB_SLAM3.so'),
    os.path.join('Vocabulary', 'ORBvoc.txt'),
)


def default_prefix() -> str:
    return os.environ.get(ENV_PREFIX) or DEFAULT_PREFIX


def missing_prerequisites(prefix: str | None = None) -> list[str]:
    """Return paths of required ORB-SLAM3 files that do not exist under prefix."""
    root = prefix if prefix is not None else default_prefix()
    missing: list[str] = []
    for rel in REQUIRED_FILES:
        candidate = os.path.join(root, rel)
        if not os.path.isfile(candidate):
            missing.append(candidate)
    return missing


def format_missing_warning(missing: list[str], script_path: str, prefix: str) -> str:
    """Format a clear build-time warning pointing to install_prerequisites.sh."""
    lines = [
        f'[kvn_orbslam3] WARNING: ORB-SLAM3 prerequisites not found under {prefix}:',
        *[f'  - missing: {path}' for path in missing],
        f'  Install them with: {script_path}',
        f'  (Or set {ENV_PREFIX}=/path/to/orbslam3 if installed elsewhere.)',
        '  Continuing build: the default slam_toolbox backend is unaffected.',
    ]
    return '\n'.join(lines) + '\n'


def warn_if_missing(
    script_path: str,
    prefix: str | None = None,
    stream: TextIO | None = None,
) -> list[str]:
    """Warn on stream (default stderr) if ORB-SLAM3 prerequisites are missing."""
    root = prefix if prefix is not None else default_prefix()
    missing = missing_prerequisites(root)
    if missing:
        out = stream if stream is not None else sys.stderr
        out.write(format_missing_warning(missing, script_path, root))
        out.flush()
    return missing
