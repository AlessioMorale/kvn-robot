# kvn_orbslam3

Optional **ORB-SLAM3 (RGB-D + IMU)** SLAM backend for the KVN rover (`slam_backend:=orbslam3`), replacing `slam_toolbox` while keeping `kvn_ekf`, Nav2, `explore_lite` and `kvn_orchestrator` interfaces unchanged.

## Architecture

```
OAK-D-Lite (/oak/rgb/image_raw, /oak/stereo/image_raw) + ISM330DHCX (/imu @ 208 Hz)
        │
        ▼
  orbslam3_rgbd_inertial (external C++ ROS 2 wrapper)
        │  /orbslam3/camera_pose (PoseStamped in map frame)
        ▼
  orbslam3_tf_bridge ──▶ TF: map -> odom  (kvn_ekf owns odom -> base_link)
        │                /orbslam3/base_pose, /orbslam3/tracking_status, /diagnostics
        ▼
  orbslam3_occupancy_mapper (/scan + map->lidar_attachment_link) ──▶ /map, /map_metadata
```

- **TF tree preserved:** `kvn_ekf` (`fuse`) still publishes `odom -> base_link`. `orbslam3_tf_bridge` publishes `map -> odom` (flattened to SE(2) by default so VIO roll/pitch/z noise does not tilt Nav2's 2D costmaps).
- **2D `/map` preserved:** `orbslam3_occupancy_mapper` integrates `/scan` into a fixed-size log-odds grid (~800 KB for 20×20 m at 5 cm resolution) so Nav2 `static_layer`, `explore_lite` and `map_saver_server` work identically to `slam_toolbox`.
- **Fail-safe tracking loss:** When `/orbslam3/camera_pose` stops updating past `pose_timeout_s` (1.5 s), `orbslam3_tf_bridge` stops broadcasting `map -> odom` so `kvn_orchestrator`'s `TransformFresh` blocks autonomy.

## Raspberry Pi 5 Envelope (`orbslam3.md`)

Running CPU-only ORB-SLAM3 on a Raspberry Pi 5 (quad-core Cortex-A76) pushes compute near its ceiling (2.5–3 cores during Loop Closing / Local BA):

- **Active cooling required:** Official Raspberry Pi Active Cooler (throttling breaks real-time tracking within 60–90 s).
- **Memory:** 8 GB Pi 5 recommended for large maps (ORB-SLAM3 retains keyframes/map points in RAM); `orbslam3_occupancy_mapper` uses a fixed-bound array.
- **Sensor & algorithm caps** (enforced in `config/oak_d_lite_rgbd_inertial_pi5.yaml` and `kvn_robot_bringup/config/oak_orbslam3.yaml`):
  - OAK-D-Lite aligned RGB-D at `640×400`, `15 Hz`, `i_subpixel: false`, `i_nn_type: none`
  - ISM330DHCX IMU at `208 Hz`
  - `ORBextractor.nFeatures: 700` (envelope: 600–800), `ORBextractor.nLevels: 6` (envelope: 4–6), `LocalMapping.LocalWindowSize: 8`, `Viewer.UseViewer: 0`

## Prerequisites & Build

Install the C++ prerequisites (`apt` deps, Pangolin `v0.9.1`, `ORB_SLAM3` in `/opt/orbslam3`, and `Vocabulary/ORBvoc.txt`) with the helper script:

```bash
# Preview commands first
./src/kvn-robot/kvn_orbslam3/scripts/install_prerequisites.sh --dry-run

# Run on Raspberry Pi 5 (auto-caps to --jobs 2 on 4 GB RAM; optionally clone a ROS 2 wrapper)
./src/kvn-robot/kvn_orbslam3/scripts/install_prerequisites.sh --jobs 2 \
  --wrapper-repo https://github.com/zang09/ORB_SLAM3_ROS2.git
```

Then build the workspace:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to kvn_orbslam3 kvn_robot_bringup
```

## Running

```bash
# Real robot
ros2 launch kvn_robot_bringup robot.launch.py slam_backend:=orbslam3

# Simulation
ros2 launch kvn_robot_bringup sim_nav_stack.launch.py slam_backend:=orbslam3

# Systemd (set in /etc/kvn/kvn-ros.env)
KVN_SLAM_BACKEND=orbslam3
KVN_ORBSLAM3_VOCAB=/opt/orbslam3/Vocabulary/ORBvoc.txt
```

An external ROS 2 ORB-SLAM3 wrapper (default package `orbslam3`, executable `rgbd-inertial`) and `ORBvoc.txt` must be installed on the target machine.

## Tests

```bash
PYTHONPATH=src/kvn-robot/kvn_orbslam3 python3 -m unittest discover -s src/kvn-robot/kvn_orbslam3/test -p 'test_*.py' -v
```
