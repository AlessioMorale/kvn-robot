#!/usr/bin/env bash
# Install ORB-SLAM3 C++ prerequisites (apt deps, Pangolin, ORB_SLAM3, ORBvoc.txt)
# for the KVN rover on Raspberry Pi 5 / Ubuntu (ROS 2 Lyrical).
#
#   ./install_prerequisites.sh                  # full install into /opt/orbslam3
#   ./install_prerequisites.sh --jobs 2         # cap parallel jobs (recommended on 4 GB Pi 5)
#   ./install_prerequisites.sh --dry-run        # print commands without modifying the system
#   ./install_prerequisites.sh --wrapper-repo https://github.com/zang09/ORB_SLAM3_ROS2.git
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

prefix="/opt/orbslam3"
jobs=""
skip_apt=0
skip_pangolin=0
skip_orbslam3=0
wrapper_repo=""
ws=""
dry_run=0

usage() {
    cat <<EOF
Usage: $0 [options]

  --prefix PATH        install directory for ORB-SLAM3 and vocabulary (default: /opt/orbslam3)
  --jobs N             parallel build jobs (default: 2 if RAM < 6 GB, else min(nproc, 4))
  --skip-apt           skip apt-get system dependency installation
  --skip-pangolin      skip building/installing Pangolin v0.9.1
  --skip-orbslam3      skip cloning/building ORB_SLAM3 and extracting ORBvoc.txt
  --wrapper-repo URL   optional ROS 2 ORB-SLAM3 wrapper git URL to clone into <ws>/src/orbslam3_ros2
  --ws PATH            colcon workspace root (default: first parent directory containing src/)
  --dry-run            print commands that would run without making changes
  -h, --help           show this help
EOF
}

die() { echo "install_prerequisites.sh: $*" >&2; exit 1; }

run() {
    if [[ $dry_run -eq 1 ]]; then
        printf '[dry-run]'
        printf ' %q' "$@"
        printf '\n'
    else
        "$@"
    fi
}

detect_default_jobs() {
    local mem_kb=0
    if [[ -r /proc/meminfo ]]; then
        mem_kb="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
    elif command -v sysctl >/dev/null 2>&1; then
        local mem_bytes
        mem_bytes="$(sysctl -n hw.memsize 2>/dev/null || echo 0)"
        mem_kb=$((mem_bytes / 1024))
    fi

    # 6 GB threshold (6 * 1024 * 1024 KB) keeps 4 GB Pi 5 builds safe from OOM
    if [[ "$mem_kb" -gt 0 && "$mem_kb" -lt 6291456 ]]; then
        echo 2
        return
    fi

    local cpus=2
    if command -v nproc >/dev/null 2>&1; then
        cpus="$(nproc)"
    elif command -v sysctl >/dev/null 2>&1; then
        cpus="$(sysctl -n hw.ncpu 2>/dev/null || echo 2)"
    fi
    if [[ "$cpus" -gt 4 ]]; then
        echo 4
    else
        echo "$cpus"
    fi
}

find_workspace() {
    local dir="$SCRIPT_DIR"
    while [[ "$dir" != "/" ]]; do
        if [[ -d "$dir/src" ]]; then
            echo "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix) prefix="${2:?--prefix needs a path}"; shift 2 ;;
        --jobs) jobs="${2:?--jobs needs a number}"; shift 2 ;;
        --skip-apt) skip_apt=1; shift ;;
        --skip-pangolin) skip_pangolin=1; shift ;;
        --skip-orbslam3) skip_orbslam3=1; shift ;;
        --wrapper-repo) wrapper_repo="${2:?--wrapper-repo needs a URL}"; shift 2 ;;
        --ws) ws="${2:?--ws needs a path}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

if [[ -z "$jobs" ]]; then
    jobs="$(detect_default_jobs)"
fi
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer, got: $jobs"

if [[ -z "$ws" ]]; then
    ws="$(find_workspace || true)"
fi

echo "ORB-SLAM3 prerequisites: prefix=$prefix jobs=$jobs ws=${ws:-<not detected>} dry_run=$dry_run"

# 1. System packages
if [[ $skip_apt -eq 0 ]]; then
    if [[ $dry_run -eq 0 ]] && ! command -v apt-get >/dev/null 2>&1; then
        die "apt-get not found; pass --skip-apt on non-Debian/Ubuntu systems"
    fi
    run sudo apt-get update
    run sudo apt-get install -y \
        build-essential cmake git pkg-config \
        libeigen3-dev libopencv-dev libssl-dev \
        libboost-serialization-dev libglew-dev libpython3-dev \
        libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev
fi

# 2. Pangolin v0.9.1
if [[ $skip_pangolin -eq 0 ]]; then
    if [[ -f /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake ]]; then
        echo "Pangolin already installed at /usr/local/lib/cmake/Pangolin; skipping."
    else
        pangolin_dir="$prefix/thirdparty/Pangolin"
        if [[ ! -d "$pangolin_dir" ]]; then
            run sudo mkdir -p "$prefix/thirdparty"
            run sudo chown -R "$USER:$USER" "$prefix"
            run git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin.git "$pangolin_dir"
        fi
        run cmake -S "$pangolin_dir" -B "$pangolin_dir/build" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_TESTS=OFF \
            -DBUILD_PANGOLIN_PYTHON=OFF
        run cmake --build "$pangolin_dir/build" -j"$jobs"
        run sudo cmake --install "$pangolin_dir/build"
    fi
fi

# 3. ORB-SLAM3 C++ library + Vocabulary/ORBvoc.txt
if [[ $skip_orbslam3 -eq 0 ]]; then
    if [[ ! -d "$prefix" ]]; then
        run sudo mkdir -p "$prefix"
        run sudo chown -R "$USER:$USER" "$prefix"
    fi

    if [[ ! -f "$prefix/CMakeLists.txt" ]]; then
        if [[ -d "$prefix/thirdparty" ]]; then
            # Preserve thirdparty/ if Pangolin was cloned there before ORB_SLAM3
            orbslam_tmp="$prefix/.orbslam3_clone"
            run git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$orbslam_tmp"
            run bash -c "shopt -s dotglob && mv '$orbslam_tmp'/* '$prefix'/ && rmdir '$orbslam_tmp'"
        else
            run git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$prefix"
        fi
    fi

    for tp in DBoW2 g2o Sophus; do
        tp_dir="$prefix/Thirdparty/$tp"
        run cmake -S "$tp_dir" -B "$tp_dir/build" -DCMAKE_BUILD_TYPE=Release
        run cmake --build "$tp_dir/build" -j"$jobs"
    done

    run cmake -S "$prefix" -B "$prefix/build" -DCMAKE_BUILD_TYPE=Release
    run cmake --build "$prefix/build" -j"$jobs"

    if [[ ! -f "$prefix/Vocabulary/ORBvoc.txt" ]]; then
        run tar -xf "$prefix/Vocabulary/ORBvoc.txt.tar.gz" -C "$prefix/Vocabulary"
    fi

    ld_conf="/etc/ld.so.conf.d/orbslam3.conf"
    if [[ $dry_run -eq 1 ]]; then
        echo "[dry-run] write $ld_conf ($prefix/lib, $prefix/Thirdparty/DBoW2/lib, $prefix/Thirdparty/g2o/lib)"
        run sudo ldconfig
    else
        printf '%s\n' \
            "$prefix/lib" \
            "$prefix/Thirdparty/DBoW2/lib" \
            "$prefix/Thirdparty/g2o/lib" \
            | sudo tee "$ld_conf" >/dev/null
        sudo ldconfig
    fi
fi

# 4. Optional ROS 2 wrapper clone
if [[ -n "$wrapper_repo" ]]; then
    [[ -n "$ws" ]] || die "could not detect colcon workspace; pass --ws PATH"
    wrapper_dir="$ws/src/orbslam3_ros2"
    if [[ -d "$wrapper_dir" ]]; then
        echo "ROS 2 wrapper already exists at $wrapper_dir; skipping clone."
    else
        run git clone "$wrapper_repo" "$wrapper_dir"
    fi
fi

cat <<EOF

Next steps (from your colcon workspace):
  source /opt/ros/lyrical/setup.bash
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install --packages-up-to kvn_orbslam3 kvn_robot_bringup
  source install/setup.bash
  ros2 launch kvn_robot_bringup robot.launch.py slam_backend:=orbslam3
EOF
