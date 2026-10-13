# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The bring-up the three teleoperation launches share.

Three entry points, one bring-up: controlling the arm, the gripper, or both is
the same stack with a different set of components loaded and a different list of
controllers spawned. Writing it once here is what keeps the three files short
enough to read, and what makes the difference between them visible in one place:

    teleop_arm.launch.py       the arm, teleoperated
    teleop_gripper.launch.py   the gripper, driven by the trigger
    teleop_arm_gripper.launch.py   both

The one switch
--------------
`dry_run` says whether this run drives hardware or simulates it, and it is the
only mode argument there is:

    dry_run:=true   (default) every component in the run is a simulation of itself
    dry_run:=false  the components are the hardware drivers

Simulation means ros2_control's mock_components: what the controller writes is
what the state reads, nothing opens a serial port, nothing opens can0. Hardware
means the two packages' own components - the arm's direct-USB one and the
gripper's - which is why this is a decision and not a default: a launch file
cannot know what is plugged in, so it moves nothing unless it is told to.

What each launch does with it is not identical, and that is deliberate rather than
an oversight:

    the arm launch        the arm follows `dry_run`; the gripper component is
                          simulated whatever it says, because this launch does not
                          control it and a real gripper holding still is still a
                          motor being driven.
    the gripper launch    the arm is always simulated - the teleoperation node
                          needs a joint state to hold and to plan from, and with the
                          arm absent the simulated one is what provides it - while
                          the gripper follows `dry_run`. This is the bench with a
                          gripper and no arm.
    the combined launch   both follow `dry_run`.

`hardware_arguments` is that mapping as a function, because it is the part worth
testing: everything else here is plumbing.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "litearm_vr_teleop"

#: Everything this stack starts shares one domain, and it is not 0: domain 0 is
#: what other equipment on the network uses. Localhost-only because the headset is
#: reached over its own socket, not over DDS. The environment wins when it says
#: something, so a rig that runs two of these side by side - or a test - can put
#: one of them somewhere else without this file being edited.
DEFAULT_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "42")
DEFAULT_LOCALHOST_ONLY = os.environ.get("ROS_LOCALHOST_ONLY", "1")

#: How long the daemon-free hardware component is given before the controller
#: manager starts. The component opens its serial port while it configures, and
#: starting both at once makes the control node lose the race and take the stack
#: down with it. Two seconds is margin rather than a measurement.
HARDWARE_HEAD_START_S = 2.0

_TRUE = ("1", "true", "yes", "on")


def is_true(value):
    """Read a launch argument that is meant to be a flag."""
    return str(value).strip().lower() in _TRUE


def hardware_arguments(dry_run, arm, gripper):
    """The xacro arguments that decide what each hardware component drives.

    Pure on purpose: the mapping from the one switch to the two components'
    simulation flags is the whole of the mode logic, and a test can hold it to
    account without launching anything.
    """
    # The arm: real only when this launch drives it and the run is real. A launch
    # that leaves the arm out still loads a component for it - the description is
    # one robot - so the arm is simulated in that case too.
    arguments = {"use_mock_hardware": "false" if (arm and not dry_run) else "true"}

    # The gripper: the same rule, and the motor gate follows it, because "drive the
    # hardware" and "is a motor attached" are the same statement in a launch file
    # that was asked for hardware.
    real_gripper = gripper and not dry_run
    arguments["litegrip_dry_run"] = "false" if real_gripper else "true"
    arguments["litegrip_hardware_enable"] = "true" if real_gripper else "false"

    # Not a mode, and not optional: the worst case velocity the gripper's component
    # may have to damp out, which it needs before it can argue that its torque
    # budget holds. Its own default is a fail-closed negative meaning "not given",
    # and the component then refuses to send a motion frame at all - while the
    # motor is enabled and holding, and while the controller accepts every goal it
    # is given. The symptom is a real gripper that is clearly on and never moves,
    # against a mock one that moves, because the mock does not go through the
    # component. The value is the calibrated one from the arm's own launch.
    arguments["litegrip_max_feedback_velocity_rad_s"] = "1.5"
    return arguments


def controller_names(arm, gripper):
    """The controllers this run spawns, in the order they are spawned.

    One spawner invocation with the whole list, not one process per controller:
    ros2_control loads and activates them one at a time within that invocation,
    and parallel spawner processes have been observed to collide while loading
    plugins and parameters. A list is that ordering by construction.
    """
    names = ["joint_state_broadcaster"]
    if arm:
        names.append("vr_forward_controller")
    if gripper:
        names.append("gripper_controller")
    return names


def declare_mode_arguments():
    """The arguments every one of the three launches offers, and no others.

    Three, and only the first of them is a mode: `dry_run` decides simulation
    against hardware, `use_rviz` is whether to watch, and `headset_port` is where
    the headset connects. Everything else this stack can be tuned with lives in
    its parameter files, which is where a number that is not a mode belongs.
    """
    return [
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="true (default) = every component simulates itself; false = "
                        "the hardware drivers. Nothing is driven unless this is false"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="start rviz2"),
        DeclareLaunchArgument(
            "headset_port", default_value="8765",
            description="WebSocket port the headset connects to"),
    ]


def _description_package():
    path = os.path.join(
        get_package_share_directory("litearm_vr_moveit_config"), "config",
        "arm_description_package.txt")
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read().strip()


def _robot_description(hardware):
    """The one description both the hardware and the display are built from."""
    package = _description_package()
    arguments = [
        "xacro ",
        os.path.join(
            get_package_share_directory("litearm_vr_moveit_config"), "urdf",
            "litearm_vr.urdf.xacro"),
        " arm_description_xacro:=",
        os.path.join(
            get_package_share_directory(package), "urdf", package + ".urdf"),
    ]
    for name, value in sorted(hardware.items()):
        arguments.append(" %s:=%s" % (name, value))
    return Command(arguments)


def bring_up(*, arm, gripper, dry_run, use_rviz, headset_port):
    """The actions all three launches start, in the order they have to start in."""
    share = get_package_share_directory(PACKAGE)
    moveit_share = get_package_share_directory("litearm_vr_moveit_config")
    teleop_params = os.path.join(share, "config", "vr_teleop.yaml")
    controller_params = os.path.join(share, "config", "vr_controllers.yaml")

    hardware = hardware_arguments(dry_run, arm, gripper)
    # Wrapped as strings, not passed as they are: a description is a long XML
    # document, and launch's parameter reader tries YAML on anything it is given
    # unless it is told the type - which is what the old XML launch said with
    # type="str" and what ParameterValue says here.
    description = ParameterValue(_robot_description(hardware), value_type=str)
    semantic = ParameterValue(
        Command(["xacro ", os.path.join(moveit_share, "srdf", "litearm_vr.srdf.xacro")]),
        value_type=str)

    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", DEFAULT_DOMAIN_ID),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", DEFAULT_LOCALHOST_ONLY),
        SetEnvironmentVariable(
            "LITEARM_VR_PYTHON", os.path.join(os.environ.get("HOME", ""), "venvs",
                                              "metavr-arm", "bin", "python")),
        # A description whose meshes live in a package this environment cannot see
        # produces a stack that runs and a display that draws nothing, with the only
        # complaint coming from rviz. One line here, before any of that. The tool
        # never fails the launch: such a stack still teleoperates the arm.
        LogInfo(msg=Command([
            os.path.join(share, "..", "..", "lib", PACKAGE, "check_description_meshes.py"),
            " ", get_package_share_directory(_description_package()),
            " ", get_package_share_directory("litegrip_description")])),
        LogInfo(msg=[
            "──────── LiteArm VR teleop ────────\n",
            "  mode: %s (dry_run:=%s)\n" % ("hardware drivers" if not dry_run else
                                             "simulation (nothing is driven)",
                                             "true" if dry_run else "false"),
            "  controlling: %s\n" % " + ".join(
                name for name, wanted in (("arm", arm), ("gripper", gripper)) if wanted),
            "  arm component: %s, gripper component: dry run %s, motor gate %s\n" % (
                "mock_components" if hardware["use_mock_hardware"] == "true"
                else "the hardware driver",
                hardware["litegrip_dry_run"], hardware["litegrip_hardware_enable"]),
            "  headset: ws://0.0.0.0:",
            LaunchConfiguration("headset_port"),
            "\n  To watch this stack from another terminal, export:\n",
            "      export ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\n" % (
                DEFAULT_DOMAIN_ID, DEFAULT_LOCALHOST_ONLY),
            "────────────────────────────────────",
        ]),
    ]

    # The hardware first, and given a head start: the component configures by
    # opening its port, and the controller manager's hardware block fails to
    # configure if it gets there first.
    actions.append(TimerAction(period=HARDWARE_HEAD_START_S, actions=[
        Node(package="controller_manager", executable="ros2_control_node", output="both",
             parameters=[{"robot_description": description}, controller_params]),
    ]))
    actions.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="both",
        parameters=[{"robot_description": description}]))

    actions.append(Node(
        package="controller_manager", executable="spawner", name="spawner", output="screen",
        arguments=["--param-file", controller_params, "--controller-manager",
                   "/controller_manager", "--controller-manager-timeout", "60"]
                  + controller_names(arm, gripper)))

    if use_rviz:
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="log",
            arguments=["-d", os.path.join(moveit_share, "rviz", "litearm_vr.rviz")],
            parameters=[{"robot_description": description,
                         "robot_description_semantic": semantic}]))

    # The link re-executes itself into the interpreter LITEARM_VR_PYTHON names
    # before importing websockets, in place, so launch's process handling and
    # SIGINT delivery are unaffected.
    actions.append(Node(
        package=PACKAGE, executable="headset_link.py", name="headset_link", output="screen",
        parameters=[teleop_params, {"port": LaunchConfiguration("headset_port")}]))

    # The teleoperation node. `gripper_enabled` is what keeps the trigger from
    # asking for a controller this run did not spawn: with no gripper in this run
    # there is no action server, and the node would warn about it once per press.
    # Both descriptions, not just the kinematic one: the node builds its planner
    # from the URDF and the SRDF together at startup, and without the semantic one
    # it logs "planning is unavailable" and holds instead of following - which
    # reads like a broken plan rather than like a missing parameter.
    actions.append(Node(
        package=PACKAGE, executable="vr_teleop_node", name="vr_teleop", output="screen",
        parameters=[teleop_params,
                    {"robot_description": description,
                     "robot_description_semantic": semantic},
                    {"gripper_enabled": bool(gripper)}]))

    return actions
