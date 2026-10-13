# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The gripper, driven by the trigger, with the arm simulated.

    ros2 launch litearm_vr_teleop teleop_gripper.launch.py                  # simulated
    ros2 launch litearm_vr_teleop teleop_gripper.launch.py dry_run:=false   # the gripper

This is the bench with a gripper and no arm, which is the normal way to work on a
grasp: the teleoperation node needs an arm to hold and to plan from, so the arm is
simulated for it, and only the gripper follows `dry_run`. False therefore means
"drive the gripper's hardware" here, and the arm's motors are not involved at all.

What the operator can do: the grip press returns the simulated arm to the start
pose and following moves nothing, because there is no arm controller in this run;
the trigger is what is being tested.
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
        arm=False,
        gripper=True,
        dry_run=is_true(LaunchConfiguration("dry_run").perform(context)),
        use_rviz=is_true(LaunchConfiguration("use_rviz").perform(context)),
        headset_port=LaunchConfiguration("headset_port").perform(context))


def generate_launch_description():
    return LaunchDescription(
        declare_mode_arguments() + [OpaqueFunction(function=_launch)])
