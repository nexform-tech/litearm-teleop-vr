# litearm-teleop-vr

VR teleoperation for the **LiteArm robotic manipulator series**: a Quest 3 headset drives the arm and its
LiteGrip gripper through ros2_control. Read this if you are building, running or reviewing the stack; the
reasoning behind each decision lives in the package READMEs, and this file is the map.

Chinese documentation: [README.zh-CN.md](README.zh-CN.md), and a five minute path through the stack:
[docs/QUICKSTART.zh.md](docs/QUICKSTART.zh.md).

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
- `websockets` for the Python node. The system Python on the reference machine does not have it, so
  `headset_link.py` re-executes itself into the interpreter named by `LITEARM_VR_PYTHON`
  (`~/venvs/metavr-arm/bin/python` by default) unless the current one already has the module.

## Build

This repository is one directory inside a colcon workspace, not a workspace of its own. Put it in
`ws/src/`, next to the sibling packages, and build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select litearm_vr_teleop litearm_vr_moveit_config
source install/setup.bash
```

## Run

Two entry points, both of which keep away from real hardware:

```bash
ros2 launch litearm_vr_teleop teleop_virtual.launch.xml   # mock hardware: nothing can move
ros2 launch litearm_vr_teleop teleop_sim.launch.xml       # real protocol, fake firmware on a pty
```

Both start the same controllers and the same nodes; the difference is the last centimetre, and it decides
what the run can tell you. The virtual launch is for watching the path and checking arithmetic. The sim
launch runs the real firmware daemon in dry run mode, which is what exercises the command stream rate, the
firmware's 100 ms watchdog and the two command interfaces. Neither launch can reach an arm: the dry run flag
is hardcoded, and the shared memory segment is named separately from the arm stack's.

A real hardware launch is a copy of `teleop_sim.launch.xml` with the dry run flag removed and the shared
memory name changed back. Read "Before real hardware" in the package README before writing one.

## Configuration

`litearm_vr_teleop/config/vr_teleop.yaml` carries the parameters that change behaviour, and three of them
are worth knowing before a session:

| Parameter | Default | What it decides |
| --- | --- | --- |
| `rotation_mode` | `world` | Whether the tool copies the controller's rotation in the world (`world`) or axis for axis in its own frame (`tool`, the previous behaviour). Settable while the node runs. |
| `reference_pose` | `commanded` | Whether the hand's increment is measured against the measured pose (each frame rebuilt from where the arm is, which feeds the solver's own residual back into the target and makes a still hand oscillate) or against the last commanded pose, which is the default: no residual in the loop, and the operator's whole displacement is delivered even while the arm lags, bounded by the lead cap. |
| `control_rate_hz` | `100.0` | The rate the node aims for. It does not reach it while it solves inverse kinematics per command: about 23 Hz was measured on the reference machine, which is why every `dt`-scaled limit in the config is worth checking against `ros2 topic hz /vr_forward_controller/commands`. |

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
