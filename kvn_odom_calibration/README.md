# kvn_odom_calibration

Automatic calibration of the wheel odometry parameters shared by `skid_steer_controller` and `kvn_odometry`:

- `wheel_radius`, from straight runs
- `chi`, from spins in place (effective track width = `track_width * chi`, which absorbs skid-steer slip)

The node drives the rover itself. It alternates straight runs forward and back, then spins counter-clockwise and clockwise, so the rover ends up roughly where it started. For each run it integrates the wheel rotation from `/joint_states` and measures the true motion with the lidar. It then fits both parameters by least squares.

## Running it

Needs the drive base and the lidar. Fuse, SLAM and Nav2 are not needed. Stop Nav2 and teleop so nothing else publishes `/cmd_vel`.

```bash
# Robot
ros2 launch kvn_robot_bringup robot_base.launch.py
ros2 launch kvn_robot_bringup robot_sensors.launch.py
ros2 launch kvn_odom_calibration odom_calibration.launch.py

# Simulation
ros2 launch kvn_robot_bringup sim_headless.launch.py world:=$(ros2 pkg prefix kvn_robot_description)/share/kvn_robot_description/worlds/kvn_robot.world x:=-3.5 y:=0.0
ros2 launch kvn_odom_calibration odom_calibration.launch.py use_sim_time:=true
```

Place the rover with `straight_distance` plus about 0.35 m of free floor in front of it and behind it. Where there is less, that run is shortened to the free space (the log says so), down to `min_straight_distance` (0.4 m), below which the calibration aborts. Shorter runs are a little less accurate. Choose a spot where the lidar sees walls in more than one direction; a long featureless corridor does not work. Before each run the node checks the scan, and a spin aborts if something is within about 0.2 m of the rover. Ctrl-C stops the rover.

The results are logged and written to `~/.ros/odom_calibration/`:

- `odom_calibration_<time>.yaml`: the fitted `wheel_radius` and `chi` for both nodes. Copy them into `kvn_robot_bringup/config/controllers.yaml`.
- `odom_calibration_<time>.csv`: one row per run, with wheel rotation, lidar motion, gyro yaw and scan-match quality.

With `apply_results:=true` the values are also set on the running nodes, so you can check them by driving right away. That change is lost on restart.

The log also reports two diagnostics:

- **Gyro scale vs lidar**: how well the IMU's integrated yaw matches the lidar, where 1.0 is exact.
- **Heading drift on straight runs**: a left/right wheel imbalance that a single radius cannot correct.

## How the motion is measured

**`reference: scan` (default).** The rover stands still for `settle_time` before and after each run. The node grabs one scan at each end and matches the two with a point-to-line ICP (`scan_matching.py`, numpy only). The initial guess comes from the gyro (bias re-estimated during every stop) and from the wheels. That is what lets a full 360° spin resolve to "one turn plus a few degrees". A run is discarded if fewer than `min_inlier_ratio` of the points match. A straight run is also discarded if its translation is poorly constrained (`min_observability`, for example along a corridor). A spin is discarded if the scan match and the gyro disagree by more than `max_gyro_disagreement`, which catches a match that locked onto the wrong angle.

**`reference: tf`.** The node reads `reference_frame → base_link` from TF instead, for example `map` from slam_toolbox (`nav_stack.launch.py use_nav2:=false`) or a ground-truth frame in simulation. slam_toolbox works, but it is a weaker reference here:

- It only processes a scan after `minimum_travel_distance` (0.5 m) or `minimum_travel_heading` (0.5 rad). For the tail end of each run, `map → base_link` is the last correction plus odometry, which partly calibrates odometry against itself.
- Its scan matcher starts from odometry (through fuse), and loop closures can move the graph between the start and end reads.
- It gives no per-measurement quality to reject bad runs on.

If you do use it, set both minimum travel thresholds to 0 for the calibration so that the scans at each stop get processed.
