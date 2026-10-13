# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The arm, teleoperated: hardware, controllers, the headset link and the node.

    ros2 launch litearm_vr_teleop teleop_arm.launch.py                  # simulated
    ros2 launch litearm_vr_teleop teleop_arm.launch.py dry_run:=false   # the arm

`dry_run` is the only mode argument. True (the default) simulates everything -
ros2_control mock_components, so the arm answers commands without a serial port
being opened - and false loads the real hardware driver, which enables the motors
at configure, so the e-stop should be in reach.

The gripper is not part of this run: no gripper controller is spawned and the node
has its trigger disabled. Its component is still in the description, and it stays
simulated whatever `dry_run` says, because a launch that does not control the
gripper has no business opening its motor gate.
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
        gripper=False,
        dry_run=is_true(LaunchConfiguration("dry_run").perform(context)),
        use_rviz=is_true(LaunchConfiguration("use_rviz").perform(context)),
        headset_port=LaunchConfiguration("headset_port").perform(context))


def generate_launch_description():
    return LaunchDescription(
        declare_mode_arguments() + [OpaqueFunction(function=_launch)])
