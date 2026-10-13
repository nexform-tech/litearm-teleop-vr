# litearm_vr_moveit_config

The description and solver configuration of the merged "7-axis LiteArm + LiteGrip
gripper" robot.

This package does no planning: no `move_group`, no OMPL pipeline, no controller
list, no planning joint limits. What the teleoperation stack needs from it is a
`robot_description`, a `robot_description_semantic` (the planning group is defined
in it, and the command array's joint order is pinned to it), and the solver tuning
the `plugin` backend reads. The `moveit_config` in the name is history: what is
left are two files in MoveIt's formats - the SRDF and `config/kinematics.yaml` -
while the URDF, the RViz view and the description-package detection are this
package's own work. To plan with MoveIt, the arm's own `litearm_moveit_config` is
the entrance; it carries the OMPL and controller configuration `move_group` needs.

The reasoning behind every choice here - which hardware blocks the description
emits, which values are duplicated and why, what the collision matrix keeps - is
written up in [README.zh.md](README.zh.md). This file is a pointer, not a
replacement.

## One thing worth knowing before you build it

The arm description package is `litearm_description`, and xacro cannot compose an
include path from a variable, so the name is detected at configure time through the
ament index and recorded in `config/arm_description_package.txt` for the launch
files to read.

Detection is by **generation**, not only by name: the package that provides the
geometry must declare `joint1..joint7` and `ee_link`, because that is what this
package, `litearm_ros2_control`'s hardware block and `litearm_moveit_config`'s SRDF
are all written against. The superseded CAD export of this arm — joints
`Joint1..Joint7`, links `Link1..Link7`, flange `ee_frame_link` — is a **different
arm**, not a naming variant (0 of 7 joint origins and 3.5305 kg vs 2.7911 kg of
total mass), and it is rejected at configure time with a message that says so,
rather than producing an URDF that urdfdom refuses with
`parent link [ee_link] of joint [gripper_mount_joint] not found`.

If the detected name is wrong for your workspace, override it:

    ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py arm_description_package:=<name>
