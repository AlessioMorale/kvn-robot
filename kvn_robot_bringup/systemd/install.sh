#!/usr/bin/env bash
# Install the KVN systemd units and modes for the user who runs this script.
#
#   sudo ./install.sh                  # install / update the units, keep the enabled boot mode
#   sudo ./install.sh --mode nav       # ...and enable + start kvn-nav.target as the boot mode
#   sudo ./install.sh --foxglove control   # ...and use the bridge that allows teleop + Nav2 goals
#   ./install.sh --dry-run             # show the rendered units, change nothing (no root needed)
#
# The unit files in this directory are examples written for one robot. User=, KVN_WS (kvn-ros.env)
# and the /opt/ros/<distro> path in ExecStart= are rewritten for this machine when installing.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR=/etc/systemd/system
ENV_DIR=/etc/kvn
MODES=(drive nav explore)
FOXGLOVE=(readonly control off)

usage() {
    cat <<EOF
Usage: sudo $0 [options]

  --user NAME        account the ROS services run as (default: the user who ran sudo)
  --ws PATH          colcon workspace (default: the first parent of this script with src/ and
                     install/setup.bash)
  --ros-distro NAME  ROS distro under /opt/ros (default: \$ROS_DISTRO, else the only one installed)
  --mode MODE        enable and start this boot mode: ${MODES[*]} (others are disabled)
  --foxglove ACCESS  WiFi bridge, enabled and started: readonly (kvn-foxglove), control (kvn-foxglove-control:
                     teleop and Nav2 goals from Foxglove), off. readonly/control also enable kvn-video
  --dry-run          print the rendered files and what would be done, change nothing
  -h, --help         this help
EOF
}

die() { echo "install.sh: $*" >&2; exit 1; }

user="" ws="" distro="${ROS_DISTRO:-}" mode="" foxglove="" dry_run=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) user="${2:?--user needs a name}"; shift 2 ;;
        --ws) ws="${2:?--ws needs a path}"; shift 2 ;;
        --ros-distro) distro="${2:?--ros-distro needs a name}"; shift 2 ;;
        --mode) mode="${2:?--mode needs one of: ${MODES[*]}}"; shift 2 ;;
        --foxglove) foxglove="${2:?--foxglove needs one of: ${FOXGLOVE[*]}}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

if [[ -n "$mode" && ! " ${MODES[*]} " =~ " $mode " ]]; then
    die "--mode must be one of: ${MODES[*]} (calibration is never a boot mode)"
fi
if [[ -n "$foxglove" && ! " ${FOXGLOVE[*]} " =~ " $foxglove " ]]; then
    die "--foxglove must be one of: ${FOXGLOVE[*]}"
fi

# Root is needed to install; re-run through sudo so SUDO_USER names the invoking user
if [[ $dry_run -eq 0 && $EUID -ne 0 ]]; then
    exec sudo -- "$0" "$@"
fi

# --- user -----------------------------------------------------------------------------------------
if [[ -z "$user" ]]; then
    user="${SUDO_USER:-$(id -un)}"
    [[ "$user" != root ]] || die "refusing to run the services as root: run via sudo from your account, or pass --user NAME"
fi
id "$user" >/dev/null 2>&1 || die "no such user: $user"
home="$(getent passwd "$user" | cut -d: -f6)"

# --- workspace: the first parent directory holding src/ and install/setup.bash --------------------
if [[ -z "$ws" ]]; then
    dir="$SRC_DIR"
    while [[ "$dir" != / ]]; do
        if [[ -d "$dir/src" && -f "$dir/install/setup.bash" ]]; then ws="$dir"; break; fi
        dir="$(dirname "$dir")"
    done
    [[ -n "$ws" ]] || die "no built colcon workspace above $SRC_DIR (run colcon build first, or pass --ws)"
fi
ws="$(cd "$ws" && pwd)"
[[ -f "$ws/install/setup.bash" ]] || die "$ws/install/setup.bash not found (run colcon build first)"

# --- ROS distro -----------------------------------------------------------------------------------
if [[ -z "$distro" ]]; then
    distros=(/opt/ros/*/setup.bash)
    [[ -f "${distros[0]}" ]] || die "no ROS installation under /opt/ros"
    [[ ${#distros[@]} -eq 1 ]] || die "several ROS distros under /opt/ros, pass --ros-distro"
    distro="$(basename "$(dirname "${distros[0]}")")"
fi
[[ -f "/opt/ros/$distro/setup.bash" ]] || die "/opt/ros/$distro/setup.bash not found"

echo "user:      $user ($home)"
echo "workspace: $ws"
echo "ROS:       /opt/ros/$distro"

# --- render ---------------------------------------------------------------------------------------
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
units=()
for f in "$SRC_DIR"/kvn-*.service "$SRC_DIR"/kvn-*.target; do
    # Skips dangling symlinks a --symlink-install build leaves behind for removed units
    [[ -f "$f" ]] || continue
    name="$(basename "$f")"
    sed -e "s|^User=.*|User=$user|" \
        -e "s|/opt/ros/[^/]*/setup.bash|/opt/ros/$distro/setup.bash|g" \
        "$f" > "$stage/$name"
    units+=("$name")
done
sed -e "s|^KVN_WS=.*|KVN_WS=$ws|" "$SRC_DIR/kvn-ros.env" > "$stage/kvn-ros.env"

if [[ $dry_run -eq 1 ]]; then
    echo
    echo "--- rendered files (differences from the examples in $SRC_DIR) ---"
    for name in kvn-ros.env "${units[@]}"; do
        diff -u --label "$name (example)" --label "$name (installed)" \
            "$SRC_DIR/$name" "$stage/$name" || true
    done
    echo
    echo "would install to $ENV_DIR/kvn-ros.env and $UNIT_DIR/: ${units[*]}"
    [[ -e "$ENV_DIR/calibration_$(hostname).yaml" ]] || echo "would create $ENV_DIR/calibration_$(hostname).yaml owned by $user"
    [[ -z "$mode" ]] || echo "would enable and start kvn-$mode.target, disable the other modes"
    [[ -z "$foxglove" ]] || echo "would set the Foxglove bridge to: $foxglove"
    exit 0
fi

# --- install --------------------------------------------------------------------------------------
# The earlier single kvn-wifi.service would hold TCP 8765 that kvn-foxglove needs
if [[ -e "$UNIT_DIR/kvn-wifi.service" ]]; then
    echo "removing the old kvn-wifi.service (replaced by kvn-foxglove + kvn-video)"
    systemctl disable --now kvn-wifi.service || true
    rm -f "$UNIT_DIR/kvn-wifi.service"
fi
# These are started by the mode targets only now; drop links left by an earlier per-service enable
for name in kvn-sensors.service kvn-nav.service kvn-explore.service; do
    link="$UNIT_DIR/multi-user.target.wants/$name"
    if [[ -L "$link" ]]; then
        echo "removing boot link of $name (the mode targets start it)"
        rm -f "$link"
    fi
done

install -d "$ENV_DIR"
install -m 0644 "$stage/kvn-ros.env" "$ENV_DIR/kvn-ros.env"
# Written by kvn_calibration_supervisor on SAVE; /etc/kvn stays root's, only this file is the user's
calibration="$ENV_DIR/calibration_$(hostname).yaml"
if [[ ! -e "$calibration" ]]; then
    install -m 0644 -o "$user" /dev/null "$calibration"
    echo "created $calibration (owned by $user, for the calibration supervisor)"
fi
for name in "${units[@]}"; do
    install -m 0644 "$stage/$name" "$UNIT_DIR/$name"
done
systemctl daemon-reload
echo "installed $ENV_DIR/kvn-ros.env and ${#units[@]} units in $UNIT_DIR"

if [[ -n "$mode" ]]; then
    for other in "${MODES[@]}"; do
        [[ "$other" == "$mode" ]] || systemctl disable --quiet "kvn-$other.target" || true
    done
    systemctl enable --now "kvn-$mode.target"
    echo "boot mode: kvn-$mode.target (enabled and started)"
else
    enabled="$(systemctl list-unit-files 'kvn-*.target' --state=enabled --no-legend | awk '{print $1}' | xargs)"
    echo "boot mode: ${enabled:-none enabled} (choose one with --mode ${MODES[*]// /|})"
    echo "running services are not restarted: sudo systemctl restart 'kvn-*.service' to pick up changes"
fi

# --- WiFi viewers (independent of the mode) -------------------------------------------------------
case "$foxglove" in
    readonly|control)
        if [[ "$foxglove" == control ]]; then on=kvn-foxglove-control.service off=kvn-foxglove.service
        else on=kvn-foxglove.service off=kvn-foxglove-control.service; fi
        systemctl disable --quiet "$off" || true
        systemctl enable --now "$on" kvn-video.service   # Conflicts= stops $off if it is running
        echo "foxglove: $on and kvn-video.service (enabled and started)"
        [[ "$foxglove" != control ]] || echo "  WARNING: Foxglove clients on the VPN can now drive the rover and publish to ANY topic (the bridge ignores client_topic_whitelist)"
        ;;
    off)
        systemctl disable --now --quiet kvn-foxglove.service kvn-foxglove-control.service kvn-video.service || true
        echo "foxglove: off (bridge and video disabled and stopped)"
        ;;
    *)
        enabled="$(systemctl list-unit-files kvn-foxglove.service kvn-foxglove-control.service --state=enabled --no-legend | awk '{print $1}' | xargs)"
        echo "foxglove: ${enabled:-none enabled} (choose with --foxglove ${FOXGLOVE[*]// /|})"
        ;;
esac
