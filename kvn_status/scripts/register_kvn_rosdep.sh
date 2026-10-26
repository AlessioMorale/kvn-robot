#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rule_file="${package_dir}/rosdep/kvn_python_dependencies.yaml"
source_list="/etc/ros/rosdep/sources.list.d/99-kvn.list"

if [[ ! -f "${rule_file}" ]]; then
  echo "rosdep rules not found: ${rule_file}" >&2
  exit 1
fi
if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
  echo "rosdep has not been initialized; run 'sudo rosdep init' first" >&2
  exit 1
fi

printf 'yaml file://%s\n' "${rule_file}" | sudo tee "${source_list}" >/dev/null
rosdep update
