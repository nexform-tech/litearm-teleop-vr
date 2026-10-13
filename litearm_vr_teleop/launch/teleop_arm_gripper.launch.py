# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The arm and the gripper together: what teleoperation actually is.

    ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py                  # simulated
    ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py dry_run:=false   # both real

`dry_run` is the only mode argument and it covers both devices: true (the default)
simulates them, false loads both hardware drivers - the arm's, which enables the
motors at configure, and the gripper's, which opens its motor gate. The e-stop
should be in reach for the second one, and the arm should be supported if its hold
is not what is keeping it up: the first grip press returns it to the start pose at
0.5 rad/s before anything follows.

Which of the two entry points below this one to use instead, when only one device
is on the bench, is a matter of not driving what is not there - see
teleop_arm.launch.py and teleop_gripper.launch.py.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription                                     # noqa: E402
from launch.actions import OpaqueFunction                                # noqa: E402
from launch.substitutions import LaunchConfiguration                     # noqa: E402

from teleop_common import bring_up, declare_mode_arguments, is_true      # noqa: E402


def _launch(context):
    return bring_up(
        arm=True,
        gripper=True,
        dry_run=is_true(LaunchConfiguration("dry_run").perform(context)),
        use_rviz=is_true(LaunchConfiguration("use_rviz").perform(context)),
        headset_port=LaunchConfiguration("headset_port").perform(context))


def generate_launch_description():
    return LaunchDescription(
        declare_mode_arguments() + [OpaqueFunction(function=_launch)])
