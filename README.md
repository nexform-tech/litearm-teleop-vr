# litearm-teleop-vr

VR teleoperation for the **LiteArm robotic manipulator series**: a Quest 3 headset drives the arm and its
LiteGrip gripper through ros2_control. Read this if you are building, running or reviewing the stack; the
reasoning behind each decision lives in the package READMEs, and this file is the map.

Chinese documentation: [README.zh-CN.md](README.zh-CN.md), and a five minute path through the stack:
[docs/quickstart.zh-CN.md](docs/quickstart.zh-CN.md) (Chinese),
and the same path in five minutes in English: [docs/quickstart.md](docs/quickstart.md).

## Packages

| Package | Role | Reasoning |
| --- | --- | --- |
| `litearm_vr_teleop` | The teleoperation stack: the WebSocket headset link, the teleoperation node (mapping, inverse kinematics, the command stream), and the ros2_control forward controller that drives the hardware | [README](litearm_vr_teleop/README.md) |
| `litearm_vr_moveit_config` | The MoveIt configuration for the merged arm + gripper: URDF, SRDF, kinematics, joint limits, controller list, RViz view | [README](litearm_vr_moveit_config/README.md) |

## How it works

The headset page sends one JSON frame per hand at about 72 Hz over a plain WebSocket. `headset_link.py`
does transport only: it parses the frame and publishes `VrControllerFrame`. `vr_teleop_node` owns every
teleoperation decision - grip and trigger edges, the increment mapping, the inverse kinematics, the joint
filter - and publishes seven joint positions at its own fixed rate. `VrForwardController` writes those
positions to the hardware together with the velocity that makes them reachable, because this firmware reads
the velocity field as the slew ceiling of the position reference and freezes the reference when it is zero.

Press the grip: the arm moves to the teleoperation start pose at equal joint speed, then follows the hand.
Release it: the arm holds. Pull the trigger: the gripper closes; release it: it opens. `~/stop` and
`~/start` are the external hold and resume, and a headset that stops sending frames holds the arm and
re-anchors on the next grip press.

## Requirements

- ROS 2 Humble on Ubuntu 22.04, with `moveit_core`, `ros2_control` and `pluginlib`.
- The arm's sibling packages, which this repository does not contain: `litearm_description`,
  `litearm_ros2_control`, `litearm_moveit_config`, `litegrip_description`, `litegrip_ros2_control`.
- A Quest 3 headset, or a client that speaks the same WebSocket frame format, for anything that follows.
- `websockets` for the Python node. Not every system Python has it, so `headset_link.py`
  re-executes itself into the interpreter named by `LITEARM_VR_PYTHON` unless the current one
  already has the module.

## Build

This repository is one directory inside a colcon workspace, not a workspace of its own. Put it in
`ws/src/`, next to the sibling packages, and build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select litearm_vr_teleop litearm_vr_moveit_config
source install/setup.bash
```

## Run

One entry point per bench and one switch, `dry_run`, which says whether this run drives hardware or
simulates it:

```bash
ros2 launch litearm_vr_teleop teleop_arm.launch.py            # the arm
ros2 launch litearm_vr_teleop teleop_gripper.launch.py        # the gripper, arm simulated
ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py    # both

ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py dry_run:=false   # the hardware
```

`dry_run` defaults to true, so a launch file run by accident moves nothing. False loads the hardware drivers:
the arm's component enables its motors at configure and the first grip press returns the arm to its start
pose. Read "Before real hardware" in the package README before the first of those.

## Configuration

`litearm_vr_teleop/config/vr_teleop.yaml` carries the parameters that change behaviour, and three of them
are worth knowing before a session:

| Parameter | Default | What it decides |
| --- | --- | --- |
| `control_frame` | `base_link` | Which control logic reads the controller's rotation: the robot base frame (`base_link`, the default) or the tool's own frame (`tool`, the previous behaviour). Each carries its own calibration. Settable while the node runs. |
| `reference_pose` | `commanded` | Whether the hand's increment is measured against the measured pose (each frame rebuilt from where the arm is, which feeds the solver's own residual back into the target and makes a still hand oscillate) or against the last commanded pose, which is the default: no residual in the loop, and the operator's whole displacement is delivered even while the arm lags, bounded by the lead cap. |
| `control_rate_hz` | `100.0` | The rate the node aims for. It does not reach it while it solves inverse kinematics per command: about 23 Hz has been measured with the plugin backend, which is why every `dt`-scaled limit in the config is worth checking against `ros2 topic hz /vr_forward_controller/commands`. |

## Tests

```bash
colcon test --packages-select litearm_vr_teleop
colcon test-result --verbose
```

50 tests: three C++ suites over the mapping, the point to point profile and the forward command tracker,
and two Python suites over the frame parsing and the consistency of the values that are deliberately stored
in two places.

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-teleop-isomorphic](https://github.com/nexform-tech/litearm-teleop-isomorphic) | Isomorphic leader-follower teleoperation |
| [litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control) | ros2_control hardware interface for the arm |
| [litegrip-ros2](https://github.com/nexform-tech/litegrip-ros2) | ROS 2 driver for the gripper |
| [litearm-moveit2](https://github.com/nexform-tech/litearm-moveit2) | MoveIt 2 configuration for the arm |
| [litearm-description](https://github.com/nexform-tech/litearm-description) | URDF description of the arm |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the agent operating rules in
[AGENTS.md](AGENTS.md), Conventional Commits, and automated semantic-release versioning on every merge to
`main`. All changes land through a pull request.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the [Apache License 2.0](LICENSE).
