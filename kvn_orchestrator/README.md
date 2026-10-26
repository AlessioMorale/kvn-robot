# kvn_orchestrator

A [BehaviorTree.CPP](https://www.behaviortree.dev/) (v4) orchestrator for the rover's autonomous
functions, built on the Nav2 BT API. The behaviour is defined entirely in XML: which subtrees exist,
their priority, and every topic, button index, timeout and threshold. Node implementations are
plugins loaded by name, so adding a new action means writing a plugin and referencing it from XML.

Navigation is done by Nav2's own `NavigateToPose` and `GoalReached` BT plugins
(`nav2_behavior_tree`). The orchestrator is a `nav2::LifecycleNode` that provides the blackboard
entries those plugins expect (`node`, `tf_buffer`, `bt_loop_duration`, timeouts), so any other Nav2
BT plugin (`Spin`, `BackUp`, `Wait`, ...) can be added to `plugin_libraries` and used from XML.

The shipped tree captures a home pose and provides a joystick-triggered Nav2 return-home action.

## Controls and safety

- At startup, the first localized pose is captured as home after `/odom_ekf`
  reports the rover stationary for one second and teleop is disabled.
- Hold AUX3 (`/joy.buttons[2]`) by itself to pause active exploration and navigate
  to home. Releasing AUX3, enabling teleop, or losing fresh Joy messages cancels the Nav2 goal
  (Nav2's BT node waits for Nav2 to confirm the cancellation).
- If exploration was active before return-home, cancelling the return resumes
  exploration. Successfully reaching home leaves exploration paused.
- Hold AUX3+AUX4 (`buttons[2]` and `[3]`) for 1.5 seconds to capture a replacement
  home pose. This is allowed only with teleop disabled and the rover stationary.
- Home is kept in memory (on the blackboard) and captured again after process restart.

## Tree overview

Diagrams of the shipped XML in [`trees/`](trees), using the standard behavior-tree notation. Keep them
in sync when you edit the XML.

| Node type | Symbol | Meaning |
| --- | --- | --- |
| Fallback / Selector | box with `?` | Runs children left to right; succeeds if any child succeeds |
| Sequence | box with `→` | Runs children left to right; succeeds only if all succeed |
| Condition | ellipse | Checks a true/false state |
| Action | box | A task; green, or orange when it is one of Nav2's own plugins |
| Decorator | diamond | Changes the result or timing of its single child |
| `OnAbort` | hexagon | Not standard. Two children: body, then handler if the body fails or is halted |
| `SubTree` | double-bordered box | Not standard. Reference to another tree, expanded in its own diagram |

Symbols follow Colledanchise and Ögren (reference 1). Mermaid has no true ellipse, so conditions
use its closest shape, the stadium.

The reactive variants (`ReactiveFallback`, `ReactiveSequence`) re-check their children every tick,
so a guard going false halts a running child.

### Legend

```mermaid
---
config:
  theme: base
  themeVariables:
    background: "#ffffff"
    primaryColor: "#f3f4f6"
    primaryTextColor: "#111827"
    primaryBorderColor: "#6b7280"
    lineColor: "#374151"
    edgeLabelBackground: "#ffffff"
    fontFamily: sans-serif
    fontSize: 11px
  flowchart:
    nodeSpacing: 12
    padding: 4
---
flowchart LR
    l1["Fallback<br/>?"]:::ctl ~~~ l2["Sequence<br/>→"]:::ctl ~~~ l3(["Condition"]):::cond ~~~ l4["Action"]:::act ~~~ l5["Nav2"]:::nav ~~~ l6{"Decorator"}:::dec ~~~ l7{{"OnAbort"}}:::ctl ~~~ l8[["SubTree"]]:::ctl

    classDef ctl fill:#e5e7eb,stroke:#6b7280,color:#111;
    classDef cond fill:#dbeafe,stroke:#2563eb,color:#111;
    classDef act fill:#dcfce7,stroke:#16a34a,color:#111;
    classDef nav fill:#ffedd5,stroke:#ea580c,color:#111;
    classDef dec fill:#fef9c3,stroke:#ca8a04,color:#111;
```

### Top level (`kvn_orchestrator.xml`)

```mermaid
---
config:
  theme: redux-dark
  look: classic
---
flowchart TD
    root["ReactiveFallback<br/>KVN rover orchestrator<br/>?"]:::ctl
    root --> once{"RunOnce"}:::dec
    once --> st0["SetStatus starting_up"]:::act
    root --> sethome[["SubTree SetHome<br/>AUX3+AUX4 held 1.5 s"]]:::ctl
    root --> ret[["SubTree ReturnHome<br/>AUX3 held"]]:::ctl
    root --> startup[["SubTree CaptureStartupHome<br/>no home pose yet"]]:::ctl
    root --> idle["AlwaysSuccess<br/>Idle"]:::act

    classDef ctl fill:#e5e7eb,stroke:#6b7280,color:#111;
    classDef cond fill:#dbeafe,stroke:#2563eb,color:#111;
    classDef act fill:#dcfce7,stroke:#16a34a,color:#111;
    classDef nav fill:#ffedd5,stroke:#ea580c,color:#111;
    classDef dec fill:#fef9c3,stroke:#ca8a04,color:#111;
```

### `SetHome`

```mermaid
---
config:
  theme: redux-dark
  look: classic
---
flowchart TD
    seq["ReactiveSequence<br/>Set home<br/>→"]:::ctl
    seq --> c1(["JoyButton released<br/>teleop, button 0"]):::cond
    seq --> c2(["JoyButton pressed<br/>chord 2;3, hold 1.5 s"]):::cond
    seq --> c3(["RobotStationary<br/>hold 1 s"]):::cond
    seq --> once{"OncePerPress<br/>chord 2;3"}:::dec
    once --> fb["Fallback<br/>?"]:::ctl
    fb --> s1["Sequence<br/>Capture home<br/>→"]:::ctl
    fb --> s2["Sequence<br/>Wait for localization<br/>→"]:::ctl
    s1 --> a1["CaptureRobotPose<br/>map to base_link, pose=home"]:::act
    s1 --> a2["SetStatus home_set"]:::act
    s2 --> a3["SetStatus waiting_for_localization"]:::act
    s2 --> a4["AlwaysFailure"]:::act

    classDef ctl fill:#e5e7eb,stroke:#6b7280,color:#111;
    classDef cond fill:#dbeafe,stroke:#2563eb,color:#111;
    classDef act fill:#dcfce7,stroke:#16a34a,color:#111;
    classDef nav fill:#ffedd5,stroke:#ea580c,color:#111;
    classDef dec fill:#fef9c3,stroke:#ca8a04,color:#111;
```

### `CaptureStartupHome`

Same capture fallback as `SetHome`, but it only runs while no home pose exists and has no chord or
latch.

```mermaid
---
config:
  theme: redux-dark
  look: classic
---
flowchart TD
    seq["ReactiveSequence<br/>Capture startup home<br/>→"]:::ctl
    seq --> inv{"Inverter<br/>Home pose missing"}:::dec
    inv --> c0(["EntryExists home"]):::cond
    seq --> c1(["JoyButton released<br/>teleop, button 0"]):::cond
    seq --> c2(["RobotStationary<br/>hold 1 s"]):::cond
    seq --> fb["Fallback<br/>?"]:::ctl
    fb --> s1["Sequence<br/>Capture home<br/>→"]:::ctl
    fb --> s2["Sequence<br/>Wait for localization<br/>→"]:::ctl
    s1 --> a1["CaptureRobotPose<br/>map to base_link, pose=home"]:::act
    s1 --> a2["SetStatus startup_home_captured"]:::act
    s2 --> a3["SetStatus waiting_for_localization"]:::act
    s2 --> a4["AlwaysFailure"]:::act

    classDef ctl fill:#e5e7eb,stroke:#6b7280,color:#111;
    classDef cond fill:#dbeafe,stroke:#2563eb,color:#111;
    classDef act fill:#dcfce7,stroke:#16a34a,color:#111;
    classDef nav fill:#ffedd5,stroke:#ea580c,color:#111;
    classDef dec fill:#fef9c3,stroke:#ca8a04,color:#111;
```

### `ReturnHome`

The guards run every tick. If one goes false while the body is running, `OnAbort` halts the body,
which makes Nav2's node cancel the goal, and then runs the handler.

```mermaid
---
config:
  theme: redux-dark-color
---
flowchart TD
    seq["ReactiveSequence<br/>Return home<br/>→"]:::ctl
    seq --> g1(["JoyButton released<br/>teleop, button 0"]):::cond
    seq --> g2(["JoyButton pressed<br/>AUX3, button 2"]):::cond
    seq --> g3(["JoyButton released<br/>AUX4, button 3"]):::cond
    seq --> g4(["EntryExists home"]):::cond
    seq --> once{"OncePerPress<br/>AUX3"}:::dec
    once --> abort{{"OnAbort<br/>body, handler"}}:::ctl

    abort -->|body| body["Sequence<br/>Navigate home<br/>→"]:::ctl
    abort -->|handler| handler["Sequence<br/>Abort handler<br/>→"]:::ctl

    body --> b1["Script<br/>nav_error := 0<br/>resume_exploration := false"]:::act
    body --> b2{"ForceSuccess<br/>Remember exploration"}:::dec
    b2 --> b2s["Sequence<br/>→"]:::ctl
    b2s --> b2c(["IsExploring"]):::cond
    b2s --> b2a["Script<br/>resume_exploration := true"]:::act
    body --> b3["PublishBool<br/>/explore/resume = false"]:::act
    body --> b4["SetStatus returning_home"]:::act
    body --> b5["WaitSeconds 0.5"]:::act
    body --> b6["NavigateToPose<br/>goal=home"]:::nav
    body --> b7(["GoalReached<br/>goal=home"]):::nav
    body --> b8["SetStatus home_reached"]:::act

    handler --> h1["Script<br/>abort_status from nav_error"]:::act
    handler --> h2["SetStatus abort_status<br/>return_rejected, return_failed<br/>or return_cancelled"]:::act
    handler --> h3{"ForceSuccess<br/>Resume exploration"}:::dec
    h3 --> h3s["Sequence<br/>→"]:::ctl
    h3s --> h3c(["ScriptCondition<br/>resume_exploration"]):::cond
    h3s --> h3a["PublishBool<br/>/explore/resume = true"]:::act

    classDef ctl fill:#e5e7eb,stroke:#6b7280,color:#111;
    classDef cond fill:#dbeafe,stroke:#2563eb,color:#111;
    classDef act fill:#dcfce7,stroke:#16a34a,color:#111;
    classDef nav fill:#ffedd5,stroke:#ea580c,color:#111;
    classDef dec fill:#fef9c3,stroke:#ca8a04,color:#111;
```

## Configuration

| What | Where |
| --- | --- |
| Tree shape, priorities, topics, buttons, timeouts, thresholds, status texts | [`trees/`](trees) XML |
| Plugin libraries, tick period, which XML file/tree to run | [`config/orchestrator.yaml`](config/orchestrator.yaml) |
| Settings Nav2's BT nodes read from node parameters (`goal_reached_tol`, timeouts) | [`config/orchestrator.yaml`](config/orchestrator.yaml) |
| Alternative XML at launch | `orchestrator_tree:=/path/to/tree.xml` |

[`trees/kvn_orchestrator.xml`](trees/kvn_orchestrator.xml) is the entry point. It `<include>`s the
subtrees in [`trees/subtrees/`](trees/subtrees) and calls each one with its parameters as
`<SubTree>` attributes, for example:

```xml
<SubTree ID="ReturnHome" _autoremap="true"
         joy_topic="/joy" teleop_buttons="0" return_buttons="2" return_blocking_buttons="3"
         navigate_action="/navigate_to_pose" explore_settle_s="0.5" ... />
```

Subtrees can be nested and reused freely (`<SubTree ID="..."/>`). With `_autoremap="true"` they share
the blackboard, which is how the captured `home` pose travels from the capture subtrees to
`ReturnHome`. Set `tree_id` in the YAML to run a different `<BehaviorTree>` from the file.

If Nav2's action server is not up yet, the node keeps retrying once per second until it is.

Once Nav2 is reachable the tree is validated when the node starts: an unknown node, a missing plugin, or a bad port value
(`joy_timeout_s="-1"`, `buttons="a"`) stops the node with a message naming the offending node.
Edit the XML and restart the node to apply changes.

### Status values

`/orchestrator/status` (transient local) carries the texts set by the `SetStatus` nodes:
`starting_up`, `waiting_for_localization`, `startup_home_captured`, `home_set`, `returning_home`,
`home_reached`, `return_cancelled`, `return_failed` (Nav2 aborted) and `return_rejected`. Rename or add values by
editing the XML.

## Node reference

Standard BehaviorTree.CPP nodes (`ReactiveSequence`, `Fallback`, `Script`, `ForceSuccess`, ...) are
available. The plugins add:

| Plugin library | Node | Kind | Ports |
| --- | --- | --- | --- |
| `kvn_bt_conditions` | `JoyButton` | condition | `topic`=/joy, `buttons` (e.g. `2;3`), `state`=pressed\|released, `hold_s`=0, `timeout_s`=1. Fails when Joy is stale. |
| | `OncePerPress` | decorator | `topic`, `buttons`. Runs its child, then fails until the buttons are released. |
| | `RobotStationary` | condition | `topic`=/odom_ekf, `timeout_s`=0.5, `hold_s`=1, `max_linear_mps`=0.03, `max_angular_rps`=0.05 |
| | `IsExploring` | condition | `topic`=/explore/status |
| | `EntryExists` | condition | `key`: blackboard entry name, e.g. `home` |
| `kvn_bt_actions` | `SetStatus` | action | `topic`=/orchestrator/status, `value`. Latched; repeats are not re-sent. |
| | `PublishBool` | action | `topic`, `value` |
| | `CaptureRobotPose` | action | `parent_frame`=map, `child_frame`=base_link, output `pose`. Fails while the TF is unavailable. |
| `nav2_behavior_tree` | `NavigateToPose` | action | Nav2's node: `goal` (PoseStamped), `server_name`, outputs `error_code_id`, `error_msg`. Halting it cancels the goal and waits for Nav2 to confirm. |
| | `GoalReached` | condition | Nav2's node: `goal`, `robot_base_frame`; tolerance is the `goal_reached_tol` parameter. Used because Nav2 reports a goal preempted by someone else as success. |
| `kvn_bt_control` | `WaitSeconds` | action | `seconds`. Like `Sleep`, but accepts any number so it can be parameterized from a SubTree. |
| | `OnAbort` | control | Two children: body and handler. The handler runs if the body fails or is halted; skipped on success. |

## Writing a plugin

1. Implement a `BT::ConditionNode`, `SyncActionNode`, `StatefulActionNode`, ... and register it with
   `BT_REGISTER_NODES(factory)`. See [`src/plugins/`](src/plugins).
2. Get ROS access through `kvn_orchestrator::get_context(config())` (the `nav2::LifecycleNode`, TF,
   shared subscriptions and publishers). For an action client, derive from
   `nav2_behavior_tree::BtActionNode` instead, like Nav2's own plugins. Implement `Preparable::prepare()` to create subscriptions/publishers
   and validate ports at startup; this also lets latched topics arrive before the first tick.
3. Build it as a shared library with `BT_PLUGIN_EXPORT` defined (see `kvn_add_bt_plugin` in
   [`CMakeLists.txt`](CMakeLists.txt)). Plugins from other packages are listed as `pkg:libname`.
4. Add the library to `plugin_libraries` and use the node in XML.

Everything runs on one single-threaded executor, so nodes need no locking.

## Integration

Install dependencies with `rosdep install --from-paths src --ignore-src -y` (`behaviortree_cpp`,
`nav2_behavior_tree` and `nav2_ros_common` come from the ROS distro). The node is included by
`kvn_robot_bringup/launch/nav_stack.launch.py` (and so by `robot.launch.py` and `kvn-nav.service`) when Nav2 is enabled. Disable it with
`use_orchestrator:=false`; it is not started when `use_nav2:=false`.

## References

1. M. Colledanchise and P. Ögren, *Behavior Trees in Robotics and AI: An Introduction*.
   Chapman & Hall/CRC Artificial Intelligence and Robotics Series, 2018.
   [arXiv:1709.00084](https://arxiv.org/abs/1709.00084)
2. P. Ögren and C. I. Sprague, "Behavior Trees in Robot Control Systems", *Annual Review of
   Control, Robotics, and Autonomous Systems*, vol. 5, 2022.
   [arXiv:2203.13083](https://arxiv.org/abs/2203.13083)
3. [BehaviorTree.CPP](https://www.behaviortree.dev/) documentation, and the
   [Nav2 behavior tree documentation](https://docs.nav2.org/behavior_trees/index.html).
