# kvn_status

Reduces the robot's health to one short status string for the ELRS handset. `crsf_joy_node` forwards it as-is over CRSF `FLIGHT_MODE` telemetry (design §4).

```bash
ros2 launch kvn_status kvn_status.launch.py            # uses config/kvn_status.yaml
ros2 run kvn_status kvn_status_node --ros-args --params-file my_status.yaml
```

Started from `kvn_robot_bringup/launch/robot_base.launch.py` (disable with `use_status:=false`).

## WLED signalling light

An optional `kvn_wled_signal` node sends the current mode to a WLED ESP32 over serial using
[`wled-adapter`](https://github.com/AlessioMorale/wled-adapter). Register its rosdep rule once on
each machine:

```bash
# Only if rosdep has not been initialized on this machine yet:
sudo rosdep init
./src/kvn-robot/kvn_status/scripts/register_kvn_rosdep.sh
rosdep install --from-paths src --ignore-src -y
```

The registration script adds the checked-out rule to the system rosdep sources and runs
`rosdep update`. The rules pin the adapter to a known Git commit and install the behavior-tree
library through rosdep's pip installer.
Run the registration script again if the workspace moves. `rosdep init` is a one-time system setup;
skip it if rosdep is already initialized.

The adapter uses the WLED serial JSON interface at 115200 baud; the ESP32 must be connected to the
rover computer. It controls segment 0 by default. Enable it in the hardware bring-up with:

```bash
ros2 launch kvn_robot_bringup robot.launch.py \
  use_wled:=true wled_port:=/dev/serial/by-id/YOUR_ESP32
```

The serial node is disabled by default so a missing ESP32 does not affect normal bring-up.
Its parameters are in [config/kvn_wled_signal.yaml](config/kvn_wled_signal.yaml).
For the systemd-managed rover, enable it in `/etc/kvn/kvn-ros.env` with
`KVN_WLED_ENABLED=true` and set `KVN_WLED_PORT` to the ESP32 device path.

While `kvn_orchestrator` runs, its signalling tree chooses the mode and publishes it on
`/orchestrator/signal` (re-sent every second); the node shows that signal while it is fresh
(`signal_timeout_s`, 3 s). Without the orchestrator (it only runs with Nav2) the node chooses
itself: critical battery, robot fault, low battery, warning, exploration, Nav2 navigation, teleop,
then standby; before the first `/robot_status` message the mode is `starting_up`. Battery limits
and the serial port come from `kvn_robot_bringup/config/robot.yaml` (12.4 V critical, 13.6 V low
for the rover's 4S pack). The teleop mode follows the configured `/joy` enable button, not
measured wheel motion. `/robot_mode` publishes the mode shown as `std_msgs/String`.

| Mode | WLED indication | Note |
|---|---|---|
| `starting_up` | Blue breathe | Change to yellow breathe |
| `standby` | Dim white | 10% cycle blink 2s |
| `teleop` | Green | 10% cycle blink 1s |
| `assisted_teleop` | Purple | 10% cycle blink 1s |
| `navigation` | Blue | 10% cycle blink 1s |
| `exploration` | Cyan | 10% cycle blink 1s |
| `returning_home` | Yellow blink | Return home (switch or battery); orchestrator only |
| `battery_low` | Amber blink | 10% cycle blink 0.5s |
| `battery_critical` | Red blink | 10% cycle blink 0.5s |
| `warning` / `fault` | Amber solid / magenta blink | 10% cycle blink 0.25s |

WLED effect IDs use the standard WLED order: 0 solid, 1 blink, 2 breathe.

## Behaviour-tree orchestrator

The `kvn_orchestrator` package runs a BehaviorTree.CPP behaviour tree (defined in XML) from
`nav_stack.launch.py` when Nav2 is enabled. Command modes, safety gates, cancellation behavior,
status values and thresholds are documented in
[kvn_orchestrator/README.md](../kvn_orchestrator/README.md). This node reads its mode through the
`mode` input type (below), so the handset shows what the rover is doing.

## Output

| Topic | Type | Notes |
|---|---|---|
| `/robot_status` | `std_msgs/String` | `<STATE>[:<CODE>]`, printable ASCII, at most 15 chars (truncated). Published at `publish_rate_hz` and immediately on change |

| STATE | Meaning |
|---|---|
| `FLT` | A fault, a stale input, or a configuration error (`FLT:CFG`) |
| `WRN` | A warning |
| `RDY` | No fault or warning; teleop disabled |
| `DRV` | No fault or warning; teleop enable (AUX1) is high |
| `RTH` | No fault or warning; returning home (AUX3 or battery), orchestrator mode |
| `EXP` | No fault or warning; exploration, orchestrator mode |
| `NAV` | No fault or warning; autonomous (Nav2), orchestrator mode |
| `AST` | No fault or warning; assisted teleop, orchestrator mode |
| `INI` | No fault or warning; orchestrator initialising (waiting for EKF, localization, ...) |

Priority is `FLT` > `WRN` > `DRV` > `RTH` > `EXP` > `NAV` > `AST` > `INI` > `RDY`: the teleop
enable switch overrides every mode, so it wins over the orchestrator's state. Among several faults (or warnings) the **oldest** active one is shown (earliest onset; equal onsets go to the first input in `inputs`).

**Staleness**: an input with no update for `timeout_s` is reported as `FLT:<stale_code>`, so a dead sensor never shows as `RDY`. An input never received is given `timeout_s` of grace after startup. Time is measured on the steady clock.

## Parameters

| Name | Default | Description |
|---|---|---|
| `status_topic` | `/robot_status` | Output topic |
| `publish_rate_hz` | `2.0` | Periodic publish rate |
| `check_period_ms` | `100` | How often staleness is checked between publishes |
| `config_fault_code` | `CFG` | Code reported when an input fails to configure (unknown type, bad parameter) |
| `inputs` | `[]` | Input names; each is configured under `<name>.*` |

ROS params files can't hold lists of dicts, so inputs are listed by name and configured under that name (see [config/kvn_status.yaml](config/kvn_status.yaml)).

### Common input parameters

| Name | Default | Description |
|---|---|---|
| `<name>.type` | none | `diagnostics`, `battery`, `joy` or `mode` |
| `<name>.topic` | per type | Topic to subscribe to |
| `<name>.timeout_s` | per type | Staleness timeout (and startup grace) |
| `<name>.code` | `<NAME>` upper case | Code shown with `WRN`/`FLT` |
| `<name>.stale_code` | `code` | Code shown when the input is stale |

### Input types

| Type | Message | Default topic / timeout | Specific parameters | Behaviour |
|---|---|---|---|---|
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | `/diagnostics`, 3 s | `name_contains` (default: the input name) | Statuses whose name contains `name_contains`: worst level wins. ERROR/STALE → `FLT`, WARN → `WRN`. Staleness counts from the last array with a matching status |
| `battery` | `sensor_msgs/BatteryState` | `/battery_state`, 3 s | `warn_voltage` (13.6), `fault_voltage` (12.4) | `voltage <= fault_voltage` or not finite → `FLT`; `<= warn_voltage` → `WRN` |
| `joy` | `sensor_msgs/Joy` | `/joy`, 1 s | `enable_button` (0) | `buttons[enable_button]` high → `DRV`, else `RDY`. Only staleness raises a fault |
| `mode` | `std_msgs/String` (latched) | `/orchestrator/mode`, 3 s | `optional` (true) | Orchestrator mode → `INI`/`AST`/`NAV`/`EXP`/`RTH`; its `fault` (autonomy blocked) → `WRN`. With `optional`, a silent orchestrator is no fault, just no state |

## Adding an input type

Derive from `StatusInput` ([inputs.hpp](include/kvn_status/inputs.hpp)), call `report(severity, code)` from the subscription callback (optionally override `operating_state()`), and register a factory in `input_factories()` in [inputs.cpp](src/inputs.cpp).

The reduction and staleness logic ([status_reducer.hpp](include/kvn_status/status_reducer.hpp)) has no ROS dependency and is unit tested in `test/test_status_reducer.cpp`; `test/test_status_node.cpp` exercises the node over topics.
