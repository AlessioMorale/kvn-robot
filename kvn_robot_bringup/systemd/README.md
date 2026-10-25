# Robot services, CPU isolation and firewall (plan T4.3, T4.4)

Each layer of the stack is its own unit, so a crash or restart in one does not take down the others, and each gets its own priority.

| File | Runs | Notes |
|---|---|---|
| `kvn-ros.env` | | Shared environment, includes `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (DDS never crosses the VPN) |
| `kvn-control.service` | `robot_base.launch.py`: ros2_control, wheel odometry, IMU, ELRS teleop, hwmon, status | Locomotion. `Nice=-5`, `CPUWeight=500`, never depends on ZeroTier |
| `kvn-sensors.service` | `robot_sensors.launch.py`: lidar, OAK camera | `Wants=kvn-control` |
| `kvn-nav.service` | `nav_stack.launch.py`: fuse, slam_toolbox, Nav2 | Localisation and navigation. `Requires=kvn-control kvn-sensors` |
| `kvn-explore.service` | `explore.launch.py`: explore_lite | `Requires=kvn-nav`. The rover drives off by itself |
| `kvn-calibration.service` | `kvn_odom_calibration` | One-shot, never at boot. Stops nav and explore while it runs |
| `kvn-foxglove.service` | `foxglove_bridge.launch.py`: read-only bridge, TCP 8765 | `Nice=10`, `CPUQuota=50%`, `After=`/`Wants=zerotier-one.service` |
| `kvn-foxglove-control.service` | `foxglove_control.launch.py`: the same bridge, but clients can also publish teleop `/cmd_vel`, Nav2 goals `/goal_pose` and `/explore/resume` | Opt-in exception to rule R1. `Conflicts=kvn-foxglove` (same port). See "Foxglove access" |
| `kvn-video.service` | `video_streamer.launch.py`: H.264 video | `Nice=10`, `CPUQuota=100%`, `Wants=kvn-webrtc-signalling` |
| `kvn-webrtc-signalling.service` | `gst-webrtc-signalling-server` on TCP 8443 for the optional WebRTC video | `Nice=10`, `CPUQuota=20%`, `Restart=always` |
| `nftables-kvn.conf` | | Table `inet kvn`: TCP 8765 and 8443 accepted on `zt*` only, dropped elsewhere; nothing else touched |
| `zerotier-flow-rules.example` | | Optional ZeroTier Central flow rules |

The control path never depends on WiFi or the VPN: `kvn-control` has no links to the WiFi units, which are crash-isolated (no `Requires`/`BindsTo`/`PartOf`). Navigation units pull in what they need (`Requires=`), so `systemctl start kvn-nav` also starts locomotion and sensors.

The `User=` lines, `KVN_WS` and `/opt/ros/<distro>` in these files are examples; `install.sh` fills in the real values. Adjust the CPU quotas (bridge + video at most 150% of 400% on 4 cores) for your robot.

## Modes (targets)

The services are grouped into four operating modes, one `.target` each. Only one mode is active at a time: starting a mode starts the services it needs and stops the ones it excludes.

| Mode target | control | sensors | nav | explore | calibration |
|---|:-:|:-:|:-:|:-:|:-:|
| `kvn-drive.target`: RC driving only | ● | | | | |
| `kvn-nav.target`: localisation and navigation | ● | ● | ● | | |
| `kvn-explore.target`: autonomous exploration | ● | ● | ● | ● | |
| `kvn-calibration.target`: one odometry calibration run | ● | ● | | | ● (exits when done) |

The WiFi viewers (the Foxglove bridge and the video, see below) are not part of any mode. They are enabled on their own and keep running in every mode. Switching mode never restarts `kvn-control`, so RC driving stays up throughout. The targets list what each mode wants. Each service carries `Conflicts=` against the modes it does not belong to: `kvn-sensors` against drive; `kvn-nav` against drive and calibration; `kvn-explore` against everything but explore. `systemctl isolate` is not used: it would also stop unrelated system services such as ssh and networking.

## Foxglove access

Run one of the two bridges; they share TCP 8765, and starting one stops the other.

- **`kvn-foxglove`** (read-only, default): view only, nothing can be sent to the rover (rule R1).
- **`kvn-foxglove-control`**: also lets Foxglove/Lichtblick command the rover:
  - **Teleop panel** on `/cmd_vel`, as `geometry_msgs/Twist`. The controller stops the rover 1 s after the last message, for example if the WiFi drops.
  - **3D panel, "Publish pose"** on `/goal_pose`, in frame `map`. Nav2 drives there; needs `kvn-nav.target` or `kvn-explore.target`.
  - **Publish panel** on `/explore/resume`, as `std_msgs/Bool`: `false` pauses exploration, `true` resumes it.

  A running Nav2 goal cannot be cancelled from Foxglove (no actions or services are exposed). Send a new goal, or `sudo systemctl start kvn-drive.target`. Teleop and Nav2 both publish `/cmd_vel`, so while a goal runs, Nav2's commands win; there is no multiplexer.

```bash
sudo ./install.sh --foxglove control     # switch to the control bridge (persistent)
sudo ./install.sh --foxglove readonly    # back to view only
sudo systemctl start kvn-foxglove-control    # or just for now, until reboot
```

> **Warning:** foxglove_bridge 3.6.2 does not enforce `client_topic_whitelist`. The parameter is set, but a client can publish to *any* topic. This was tested: a made-up `/forbidden` topic got through. The three topics above are the intent, not a guarantee. With the control bridge running, anyone who can reach TCP 8765 can drive the rover. The firewall (`nftables-kvn.conf`, ZeroTier only) and the ZeroTier flow rules are what limit who that is.

## Install

Build the workspace first, then run the script from your own account (not a root shell):

```bash
./install.sh --dry-run          # optional: show what would be installed, no root needed
sudo ./install.sh --mode nav --foxglove readonly   # units, boot mode kvn-nav, read-only bridge + video
```

`install.sh` works out where the services should run and rewrites the copies it installs:

| Value | Default | Override |
|---|---|---|
| Account in `User=` | The user who ran `sudo` (`$SUDO_USER`). Refuses root | `--user NAME` |
| `KVN_WS` in `/etc/kvn/kvn-ros.env` | The first parent directory of the script that contains `src/` and `install/setup.bash` | `--ws PATH` |
| `/opt/ros/<distro>` in `ExecStart=` | `$ROS_DISTRO`, else the only distro installed | `--ros-distro NAME` |

It writes `/etc/kvn/kvn-ros.env` and all `kvn-*.service` / `kvn-*.target` files to `/etc/systemd/system/`, then runs `daemon-reload`. `--mode drive|nav|explore` enables and starts that boot mode and disables the other two; calibration is never a boot mode. `--foxglove readonly|control|off` picks the Foxglove bridge (readonly and control also enable `kvn-video`). Without these options the current choices are kept. Run it again after changing a unit file. It does not restart running services: `sudo systemctl restart 'kvn-*.service'`. It also runs from the install space: `$(ros2 pkg prefix kvn_robot_bringup)/share/kvn_robot_bringup/systemd/install.sh`.

It also cleans up the earlier layouts. It disables and removes the old `kvn-wifi.service`, whose bridge would hold the TCP 8765 port that `kvn-foxglove` needs. It removes leftover boot links of `kvn-sensors`, `kvn-nav` and `kvn-explore` from when they were enabled one by one. Those three have no `[Install]` section now; the modes start them. `kvn-control`, `kvn-foxglove` and `kvn-video` can still be enabled on their own, so RC driving comes up even with no mode target enabled.

To change the boot mode later: `sudo ./install.sh --mode drive` (or `systemctl disable` the old target and `enable` the new one).

## Day to day

```bash
systemctl list-units 'kvn-*.target'           # current mode (the active one)
systemctl status 'kvn-*'                      # what is running
journalctl -fu kvn-nav                        # follow one layer's log

sudo systemctl start kvn-explore.target       # explore (navigation comes up if needed)
sudo systemctl start kvn-nav.target           # stop exploring, keep navigation and the map
sudo systemctl start kvn-drive.target         # RC only: lidar, SLAM, Nav2 and exploration off

# Odometry calibration: stops nav/explore, drives the rover for a few minutes, then exits.
# Give it ~1.35 m of free floor in front and behind; keep the RC sticks neutral.
sudo systemctl start kvn-calibration.target
journalctl -fu kvn-calibration                # progress, then the fitted wheel_radius / chi
ls ~/.ros/odom_calibration/                   # YAML to copy into config/controllers.yaml
sudo systemctl start kvn-nav.target           # back to navigation (not automatic)
```

Starting `kvn-calibration.target` returns immediately; the run continues in the background. `sudo systemctl start kvn-calibration` (the service) instead blocks until the run is done. To try the fitted values straight away (runtime only, lost when `kvn-control` restarts):

```bash
sudo systemctl set-environment KVN_CALIBRATION_ARGS="-p apply_results:=true"
sudo systemctl start kvn-calibration.target
sudo systemctl unset-environment KVN_CALIBRATION_ARGS   # after the run
```

## WebRTC video (optional)

Uses the gst-plugins-rs `webrtcsink`, which Ubuntu does not package. Build it for the robot's distro (see remote_controller `tools/webrtc_spike/build_plugins_resolute.sh`) and install the plugin libraries and the signalling server:

```bash
sudo install -d /opt/kvn/gst-rs/bin /opt/kvn/gst-rs/lib/gstreamer-1.0
sudo install -m 0755 gst-webrtc-signalling-server /opt/kvn/gst-rs/bin/
sudo install -m 0644 libgstrswebrtc.so libgstrsrtp.so /opt/kvn/gst-rs/lib/gstreamer-1.0/
```

Then start the streamer with `webrtc:=true`: append it to the `ros2 launch` command in `kvn-video.service` (`sudo systemctl edit --full kvn-video`), or by hand `ros2 launch kvn_video_streamer video_streamer.launch.py webrtc:=true`. The camera then stays open and `webrtcsink` encodes only while a viewer is connected. If WebRTC fails, the streamer falls back to the on-demand Foxglove stream and retries WebRTC every 10 s. The media itself flows over UDP on random ports; only TCP 8443 is filtered here, so rely on ZeroTier for the rest.

Other ROS 2 shells on the robot should `set -a; . /etc/kvn/kvn-ros.env; set +a` too.

## Firewall (nftables)

```bash
sudo nft -c -f nftables-kvn.conf                       # syntax check
sudo install -D -m 0644 nftables-kvn.conf /etc/nftables.d/kvn.conf
# make /etc/nftables.conf load it (once):
echo 'include "/etc/nftables.d/*.conf"' | sudo tee -a /etc/nftables.conf
sudo systemctl enable --now nftables.service
sudo nft list table inet kvn
```

The file deletes and recreates only `inet kvn`, so reloads are idempotent. Verify: `nc -zv <lan-ip> 8765` fails, `nc -zv <zerotier-ip> 8765` succeeds, rules survive reboot.
If ZeroTier interfaces are named differently (`ip -br link`; default `ztXXXXXXXX`), edit the `iifname` pattern.

## Measure control-path jitter (T4.3)

With the bridge streaming to a viewer and video at maximum settings:

```bash
ros2 run kvn_robot_bringup joy_cmdvel_jitter.py --duration 60 --budget-ms 5
ros2 node list    # from a laptop on the VPN: must show nothing
```
