# litearm_vr_moveit_config

MoveIt 2 configuration for the merged "7-axis LiteArm + LiteGrip gripper" robot:
URDF, SRDF, kinematics, joint limits, controller list and the RViz view.

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
