# kvn_calibration_supervisor

Guided calibration of the KVN rover, driven entirely through ROS topics, so that it can be run from the Foxglove or Lichtblick publish panel without SSH. One session covers:

| Step | Finds | How |
|---|---|---|
| `odometry` | `wheel_radius`, `chi` | [kvn_odom_calibration](../kvn_odom_calibration/README.md), run inside this node |
| `imu` | IMU roll and pitch | Gravity sampled at two headings 180° apart, which cancels the floor's tilt. The node drives the half turn itself |
| `lidar` | Lidar yaw | Line fitted to a wall the rover is square to |
| `camera` | OAK-D pitch and yaw | Plane fitted to the same wall in the depth cloud, compared with the lidar's wall normal |

The results go to `/etc/kvn/calibration_<hostname>.yaml` as ROS 2 parameters.

## Running it

It needs the drive base and the sensors, and nothing else publishing `/cmd_vel` (no Nav2, no exploration):

```bash
# Robot: calibration mode (stops nav / explore, starts the supervisor)
sudo systemctl start kvn-calibration.target

# By hand
ros2 launch kvn_robot_bringup robot_base.launch.py
ros2 launch kvn_robot_bringup robot_sensors.launch.py
ros2 launch kvn_calibration_supervisor calibration_supervisor.launch.py

# Simulation
ros2 launch kvn_robot_bringup sim_headless.launch.py world:=$(ros2 pkg prefix kvn_robot_description)/share/kvn_robot_description/worlds/kvn_robot.world x:=-3.5 y:=0.0
ros2 launch kvn_calibration_supervisor calibration_supervisor.launch.py use_sim_time:=true output_path:=/tmp/calibration_sim.yaml
```

In simulation, `sim_profile:=real` mounts the simulated sensors off their URDF frames, makes the tyres 3 % larger than nominal, and adds sensor noise. It is a calibration problem with a known answer (`kvn_robot_description/config/sim_profiles.yaml`). Spawned at `x:=3.9 y:=0.0` in `kvn_robot.world`, the rover faces the east wall from 1.0 m away, ready for the lidar and camera steps. The results compared with the injected errors:

| | `ideal` | `real`, injected | `real`, measured |
|---|---|---|---|
| Lidar yaw vs URDF | 0.00° | +2.0° | +2.06° |
| Camera pitch / yaw | −0.07° / 0.00° | +3.0° / −1.5° | +2.7–2.8° / −1.5° |
| IMU roll / pitch | 0.00° / 0.00° | +1.5° / −1.0° | +1.61° / −1.10° |

The IMU values include the profile's accelerometer bias (about 0.12°), which no static method can separate from the mounting. About 0.25° of the camera pitch is lost to depth noise acting along the camera rays, an inherent limit of fitting a plane from one viewpoint. The IMU half turn leaves the rover facing away from the wall, so in simulation run the lidar and camera steps first (`steps:=[lidar,camera]`) and the IMU in a second session. The file merges both.

The node does nothing until it gets `START`. It then waits for `NEXT` before each step, so that you can place the rover first.

### From Foxglove or Lichtblick

Use the control bridge (`kvn-foxglove-control`, see `kvn_robot_bringup/systemd/README.md`), which accepts `/calibration/command` from clients.

- **Publish panel**: topic `/calibration/command`, schema `std_msgs/msg/String`, message `{"data": "NEXT"}`. Keep one panel per command (`START`, `NEXT`, `SKIP`, `ABORT`, `SAVE`) for one-click buttons.
- **Raw Messages panel** on `/calibration/status`: shows what the rover is doing, the instruction for you, the commands it accepts right now, progress and results.

### From a shell

```bash
ros2 topic pub --once /calibration/command std_msgs/msg/String "{data: START}"
ros2 topic echo --field data /calibration/status
```

## Commands

| Command | Accepted in | Effect |
|---|---|---|
| `START` | `IDLE` | New session: clears the previous results and goes to the first step |
| `NEXT` | any `*_WAIT` | The rover is in place: run this step |
| `SKIP` | any step | Leave this step out. Stops the rover if it is moving. The saved file keeps its previous values for this step |
| `ABORT` | anywhere but `IDLE` | Stops the rover, discards the session, back to `IDLE` |
| `SAVE` | `REVIEW_AND_SAVE` | Writes the file, back to `IDLE` |

Commands are case-insensitive. A command that does not apply in the current state is ignored, and the status message says so. When a step fails, for example because no wall was found or a sensor was silent, the node goes back to that step's `*_WAIT` state and puts the reason in `error`. You can then fix the setup and send `NEXT` again, or `SKIP`. The node never needs a restart.

## The steps

```
IDLE ─START─▶ ODOM_WAIT ─NEXT─▶ ODOM_CALIBRATING ─▶ IMU_WAIT_POS_1 ─NEXT─▶ IMU_SAMPLING_POS_1
  ▲                                                                              │
  │           IMU_SAMPLING_POS_2 ◀── IMU_AUTONOMOUS_TURN ◀───────────────────────┘
  │                  │
  │                  ▼
  │           LIDAR_WAIT ─NEXT─▶ LIDAR_SAMPLING ─▶ CAMERA_WAIT ─NEXT─▶ CAMERA_SAMPLING
  │                                                                          │
  └──────SAVE / ABORT── REVIEW_AND_SAVE ◀────────────────────────────────────┘
```

`ABORT` from any state goes through `ABORT_ROUTINE`, which stops the rover and waits for the odometry run to wind down, then returns to `IDLE`. The `steps` parameter removes steps entirely.

1. **Odometry** (`ODOM_*`). This is the `kvn_odom_calibration` run, with the same space requirements: about 1.4 m free in front of and behind the rover, and walls in view in more than one direction. Its parameters come from `kvn_odom_calibration/config/odom_calibration.yaml`, with the overrides in this package's config. By default the fitted values are applied to the running drive nodes (`apply_results: true`), so the half turn below already uses the calibrated `/odom`. The status shows the run count.
2. **IMU** (`IMU_*`). Put the rover down where it can turn in place, and do not touch it until the step ends. The node:
   - averages 500 accelerometer samples, which also gives the gyro bias;
   - turns 180° (the angle is 0.8 × bias-corrected gyro + 0.2 × `/odom` yaw, slowing down near the end);
   - waits 1 s for the suspension to settle, then takes 500 more samples.

   The mounting roll and pitch are the mean of the two headings. The floor's tilt, which is half their difference, is reported too. If the rover moves while sampling, the step fails.
3. **LiDAR** (`LIDAR_*`). Square the rover to a flat wall about 1 m away. The node fits a line, by RANSAC and then least squares, to 10 scans within ±0.5 rad of straight ahead. The lidar yaw is the angle that turns the wall's normal onto base_link +x. A wall far from 1 m only gives a warning.
4. **Camera** (`CAMERA_*`). Keep the rover where it is. The node pairs scans with depth clouds using `message_filters` (approximate time sync), and works in the camera mount frame (`camera_attachment_link`, reached through the static TF of the OAK-D frames). It keeps the central part of the cloud, removes the floor plane, and fits the wall plane. Pitch and yaw are then the rotation that maps this normal onto the wall normal the lidar measured in base_link, using the lidar yaw found in step 3, or the URDF value if that step was skipped. Roll about the wall normal cannot be observed from one plane, so it is not estimated.

Both fits only accept a line or plane within `max_wall_angle` (0.5 rad) of the expected wall orientation, so the floor or a side wall cannot be mistaken for the wall.

Sensor topics are only subscribed while a step samples them. In the `WAIT` states the node does no sensor processing at all.

## Output

`REVIEW_AND_SAVE` shows the values that will be written under `progress.extrinsics`. `SAVE` merges them into the file:

```yaml
# KVN calibration of kvn, written by kvn_calibration_supervisor on 2026-11-06T10:00:00.
/**:
  ros__parameters:
    extrinsics:
      base_link_to_imu:
        translation: [-0.05, -0.03, 0.025]
        rotation_rpy: [0.0123, -0.0081, 0.0]
      base_link_to_oakd:
        translation: [0.074, 0.0, 0.1]
        rotation_rpy: [0.0, 0.0312, -0.0045]
      base_link_to_lidar:
        translation: [0.03, 0.0, 0.115]
        rotation_rpy: [0.0, 0.0, -1.5532]
odometry_node:
  ros__parameters: {wheel_radius: 0.03641, track_width: 0.125, chi: 1.3127}
skid_steer_controller:
  ros__parameters: {wheel_radius: 0.03641, track_width: 0.125, chi: 1.3127}
```

- **Translations** are not calibrated. They are copied from the current TF tree, that is from the URDF, `kvn_robot_description/config/robot_config.yaml`.
- **Rotations** come from this session where a step ran. For a skipped step, the previous value in the file is kept, or the URDF value if the file has none. Each sensor writes only the angles it measures; its other angles come from the same fallback.
- **Odometry** values are written as sections for the two drive nodes, so that the file can be passed to them as a parameters file. They are written only when the odometry step ran.
- **Re-running** merges into the existing file as a dictionary. Values are overwritten and keys are never duplicated.
- **Writing** goes through a temporary file and a rename. When the directory is not writable (`/etc/kvn` belongs to root), the node rewrites the file in place. `install.sh` creates the file for the service user.

Nothing reads this file automatically yet. The URDF still takes its angles from `kvn_robot_description/config/robot_config.yaml` (`imu_roll`, `imu_pitch`, `camera_pitch`), and the odometry values from `kvn_robot_bringup/config/controllers.yaml`. Copy the values over, or pass the file as a parameters file to the nodes that should use it.

Everything the node does is also logged to `/rosout`, and from there to the journal (`journalctl -u kvn-calibration-supervisor`).

## Parameters

All parameters are in `config/calibration_supervisor.yaml`, with comments. The main ones:

| Parameter | Default | |
|---|---|---|
| `steps` | `[odometry, imu, lidar, camera]` | Steps offered after `START` |
| `output_path` | `""` | Empty: `/etc/kvn/calibration_<hostname>.yaml` |
| `imu_topic`, `scan_topic`, `cloud_topic`, `odom_topic` | `/imu`, `/scan`, `/oak/rgbd/points`, `/odom` | The robot's topics, which are the same in simulation |
| `imu_samples` | 500 | Samples at each heading |
| `turn_gyro_weight` | 0.8 | Weight of the gyro against `/odom` during the half turn |
| `wall_distance` | 1.0 | Expected distance to the wall [m]. Only used for a warning |
| `max_wall_angle` | 0.5 | How far [rad] the wall may be from the expected orientation |

## Notes on the design

- **Topic names.** The original specification uses `/imu/data` and `/camera/depth/color/points`. This rover publishes `/imu` and `/oak/rgbd/points`, so those are the defaults; set the parameters to use other topics.
- **RANSAC.** The line and plane fits are RANSAC written with numpy, like `kvn_odom_calibration`'s scan matching, rather than Open3D. Open3D is not packaged for this ROS distro and Python 3.14. The numpy version also lets the search be restricted to wall-like orientations.
- **Threading.** The FSM runs in a 20 Hz timer. Commands are queued by their callback and handled in the FSM thread, so transitions never race. Only the odometry run, which blocks, has its own thread. `ABORT` and `SKIP` stop it through `OdomCalibration.request_abort()`.

## Tests

```bash
colcon test --packages-select kvn_calibration_supervisor && colcon test-result --verbose
```

`test_algorithms.py` covers the IMU floor compensation, the lidar yaw and the camera pitch and yaw, on synthetic data that includes floor and side-wall clutter. `test_persistence.py` covers the file layout and idempotent re-runs. `test_supervisor.py` drives the FSM with rclpy through a full session (`START`, the IMU half turn, the lidar, `SAVE`), and through `SKIP`, `ABORT`, sensor timeouts and a rover that moves during sampling.
