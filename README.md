![Slamming Works brand lockup](assets/slw-lockup.svg)

# KVN rover

Slamming Works' ROS 2 workspace for a skid-steer rover, from motor control and sensor fusion through mapping and remote operation.

## Demo

There is no rover demo video or GIF in this repository yet.

## Hardware

The workspace describes the KVN rover and its sensor frames in [kvn_robot_description](kvn_robot_description/README.md). Hardware interfaces and bring-up configuration cover the DDSM-210 drive motors, ELRS receiver, IMU, lidar, and battery monitoring. A consolidated bill of materials and schematic are not included here.

## Build and run

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

### Simulation (headless gz-sim)

Install the simulation dependencies (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`) through `rosdep` as above, then start the whole stack: the rover in a gz-sim world with a lidar, IMU and depth camera, fuse odometry, slam_toolbox and Nav2.

```bash
ros2 launch kvn_robot_bringup sim_nav_stack.launch.py
```

This runs the gz server with no GUI (`--headless-rendering`) and uses the TurtleBot3 World from `nav2_minimal_tb3_sim`, so it works over SSH or in the devcontainer. Useful arguments:

| Argument | Default | Effect |
|---|---|---|
| `gui` | `false` | Also start the gz GUI client (needs a display). |
| `use_nav2` | `true` | Set `false` to run only the simulation, fuse and slam_toolbox. |
| `world` | TurtleBot3 World | `kvn_turtlebot3_world.world` is a walled area with a 3×3 grid of pillars. For the plain 10×10 m arena with boxes, pass `world:=$(ros2 pkg prefix kvn_robot_description)/share/kvn_robot_description/worlds/kvn_robot.world x:=0.0 y:=0.0`. |
| `x`, `y` | `-2.0`, `-0.5` | Spawn position in the world. |

To run only the simulated rover, without fuse, SLAM or Nav2, use `ros2 launch kvn_robot_bringup sim_headless.launch.py publish_odom_tf:=true`.

Drive and inspect it from another terminal:

```bash
# Drive: straight, then a turn
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Check the chain: wheel odom -> fuse -> SLAM map
ros2 topic hz /odom /odom_ekf /scan /imu /oak/rgbd/points
ros2 run tf2_tools view_frames          # expect map -> odom -> base_link
ros2 topic echo /map --once --field info

# Send a Nav2 goal. The map frame starts at the rover's spawn pose, so this is 1.45 m ahead of the
# start (-2.0, -0.5), in the lane between two rows of pillars of the TurtleBot3 World
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.45, y: -0.05}, orientation: {w: 1.0}}}}"

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/kvn_map
```

### Autonomous exploration

`explore.launch.py` is a separate launch file: it adds frontier exploration (`explore_lite`) on top of a running `sim_nav_stack` (or, on the robot, `nav_stack`). It reads `/map` from slam_toolbox, picks the next frontier and sends it to Nav2; when none is left it returns to the start.

```bash
ros2 launch kvn_robot_bringup explore.launch.py use_sim_time:=true   # omit use_sim_time on the robot
ros2 topic echo /explore/status                                      # exploration_started / complete
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"   # pause (true = resume)
```

`explore_lite` is not released for ROS 2 Lyrical, so it is built from source: `vcs import < repos.vcs` from the workspace root fetches it into `src/m-explore-ros2`. Tuning is in `kvn_robot_bringup/config/explore.yaml`. The frontier markers are on `/explore/frontiers`.

In the TurtleBot3 World the lidar sees most of the arena from the start, so the map barely grows. The rover mainly visits the small unknown pockets behind the pillars. For a longer exploration run, use a larger world.

To watch it, connect Foxglove to the bridge (`foxglove_bridge.launch.py`) or run RViz on the same machine with `use_sim_time:=true`.

Everything uses `/clock`, so run any extra node with `--ros-args -p use_sim_time:=true`. The robot parameters (`wheel_radius`, `track_width`) in `controllers.yaml` must match the simulated geometry in `kvn_robot_description/config/robot_config.yaml`, otherwise sim odometry and yaw will be off.

The hardware bring-up entry point is:

```bash
ros2 launch kvn_robot_bringup robot.launch.py
```

That launch file starts real hardware components (drive motors, ELRS receiver, IMU, lidar, OAK camera), then fuse odometry, slam_toolbox and Nav2. Check the device paths and configuration before using it on a rover. Pass `use_nav2:=false` to run without Nav2.

Launch files in `kvn_robot_bringup/launch`:

| Launch file | Runs on | Starts |
|---|---|---|
| `robot` | Robot | `robot_base` + `robot_sensors` + `nav_stack`, each node started once. On the robot, systemd runs the layers as separate units instead (see `kvn_robot_bringup/systemd/README.md`). |
| `robot_base` | Robot | robot_state_publisher, ros2_control, wheel odometry, IMU, teleop, hwmon, status. |
| `robot_sensors` | Robot | RPLIDAR C1 and OAK-D-Lite depth camera. |
| `nav_stack` | Robot, or simulation with `use_sim_time:=true` | fuse, slam_toolbox, Nav2. Needs `/odom`, `/imu`, `/scan` and the point cloud from the layers above. |
| `nav2_servers`, `oak_camera`, `wheel_odometry`, `teleop`, `hardware_monitor` | Robot | One subsystem each. |
| `wifi_link`, `foxglove_bridge` | Robot (the bridge also works in simulation) | WiFi access point; read-only Foxglove/Lichtblick bridge. |
| `sim_nav_stack` | **Simulation only** | `sim_headless` plus `nav_stack`. |
| `sim_headless` | **Simulation only** | gz-sim, rover, sensor bridge, controllers, wheel odometry. |

Run `robot` alone, or its layers separately; do not start `robot` together with `robot_base`, `robot_sensors` or `nav_stack`, which would duplicate nodes.

| Package | Role |
|---|---|
| `kvn_robot_description` | Rover URDF/Xacro, meshes, frames and RViz configuration. |
| `kvn_robot_bringup` | Simulation and hardware launch files, controller and subsystem configuration. |
| `skid_steer_controller`, `kvn_odometry`, `kvn_ekf` | Drive control, wheel odometry and fused odometry. |
| `kvn_joy_teleop`, `kvn_status`, `kvn_orchestrator` | Teleoperation, health reporting, and joystick-driven behaviour-tree autonomy. |
| `kvn_video_streamer` | On-demand video to Foxglove, with optional WebRTC. |

## Status

**Partly works.** The simulation stack is in place. Hardware field testing remains; the repository does not include a field-test recording or results log.

The ROS package names are retained for compatibility with existing launch files and workspace dependencies.

## License

MIT. See the license files in the individual packages.
