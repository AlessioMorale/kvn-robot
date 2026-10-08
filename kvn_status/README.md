# kvn_status

Reduces the robot's health to one short status string for the ELRS handset. `crsf_joy_node` forwards it as-is over CRSF `FLIGHT_MODE` telemetry (design §4).

```bash
ros2 launch kvn_status kvn_status.launch.py            # uses config/kvn_status.yaml
ros2 run kvn_status kvn_status_node --ros-args --params-file my_status.yaml
```

Started from `kvn_robot_bringup/launch/controller.launch.py` (disable with `use_status:=false`).

## Output

| Topic | Type | Notes |
|---|---|---|
| `/robot_status` | `std_msgs/String` | `<STATE>[:<CODE>]`, printable ASCII, at most 15 chars (truncated). Published at `publish_rate_hz` and immediately on change |

| STATE | Meaning |
|---|---|
| `FLT` | A fault, a stale input, or a configuration error (`FLT:CFG`) |
| `WRN` | A warning |
| `DRV` | No fault or warning; teleop enable (AUX1) is high |
| `RDY` | No fault or warning; teleop disabled |

Priority is `FLT` > `WRN` > `DRV` > `RDY`. Among several faults (or warnings) the **oldest** active one is shown (earliest onset; equal onsets go to the first input in `inputs`).

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
| `<name>.type` | — | `diagnostics`, `battery` or `joy` |
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

## Adding an input type

Derive from `StatusInput` ([inputs.hpp](include/kvn_status/inputs.hpp)), call `report(severity, code)` from the subscription callback (optionally override `operating_state()`), and register a factory in `input_factories()` in [inputs.cpp](src/inputs.cpp).

The reduction and staleness logic ([status_reducer.hpp](include/kvn_status/status_reducer.hpp)) has no ROS dependency and is unit tested in `test/test_status_reducer.cpp`; `test/test_status_node.cpp` exercises the node over topics.
