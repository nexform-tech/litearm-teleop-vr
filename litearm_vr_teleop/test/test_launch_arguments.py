# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Three launches, one mode argument, and the mapping between them.

What is worth testing here is not the plumbing but the three promises:

  * there are exactly three launch files, controlling the arm, the gripper, or
    both - one entry point per bench, not a matrix of flags;
  * `dry_run` is the only mode argument in any of them, and its default simulates,
    because the mistake it prevents is a launch that moves hardware because
    somebody typed a flag they did not read;
  * the switch reaches both hardware components with the meaning it claims -
    simulation means mock_components and a simulated gripper, hardware means the
    two drivers - and the arms and grippers it does NOT control stay simulated.

The last one is a pure function in `teleop_common`, which is why it is here: it is
the whole of the mode logic, and a launch file cannot be asked about it.
"""

import importlib.util
import os

import pytest

from ament_index_python.packages import get_package_share_directory

_PACKAGE = "litearm_vr_teleop"

#: The three entry points, and what each controls.
_LAUNCHES = (
    "teleop_arm.launch.py",
    "teleop_gripper.launch.py",
    "teleop_arm_gripper.launch.py",
)

#: The only arguments they offer. `dry_run` is the mode; the other two are
#: whether to watch and where the headset connects.
_ARGUMENTS = {"dry_run", "use_rviz", "headset_port"}


def _launch_dir():
    return os.path.join(get_package_share_directory(_PACKAGE), "launch")


def _module(filename):
    """Load an installed launch file as a module, the way `ros2 launch` does."""
    path = os.path.join(_launch_dir(), filename)
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _common():
    return _module("teleop_common.py")


def _declared(names):
    """The launch arguments a description declares, by evaluating one for real."""
    from launch import LaunchContext

    context = LaunchContext()
    return {
        entity.name: entity.default_value[0].perform(context)
        for entity in _module(names).generate_launch_description().entities
        if type(entity).__name__ == "DeclareLaunchArgument"
    }


def test_the_package_ships_exactly_three_launches():
    files = sorted(name for name in os.listdir(_launch_dir()) if name.endswith(".launch.py"))
    assert files == sorted(_LAUNCHES), (
        "the launch directory holds %s; the package is meant to have one entry point per "
        "bench - arm, gripper, both" % files)


@pytest.mark.parametrize("name", _LAUNCHES)
def test_every_launch_offers_the_same_three_arguments(name):
    declared = _declared(name)
    assert set(declared) == _ARGUMENTS, (
        "%s declares %s; the only mode argument is dry_run, and everything else a number "
        "can tune belongs in a parameter file" % (name, sorted(declared)))


def test_dry_run_defaults_to_simulation():
    from launch import LaunchContext

    context = LaunchContext()
    defaults = {
        entity.name: entity.default_value[0].perform(context)
        for entity in _common().declare_mode_arguments()
    }
    assert defaults["dry_run"] == "true", (
        "dry_run defaults to %r; it must default to simulation, since a launch cannot know "
        "what is plugged in" % defaults["dry_run"])


def test_the_one_switch_drives_each_component():
    """The mapping, which is the whole of the mode logic.

    Three cases worth naming: both real; the arm real and the gripper left
    simulated, because that launch does not control it; and the gripper real with
    the arm simulated, which is the bench with a gripper and no arm.
    """
    hardware = _common().hardware_arguments

    # Simulation: nothing is driven, whatever is being controlled.
    for arm, gripper in ((True, True), (True, False), (False, True)):
        simulated = hardware(True, arm, gripper)
        assert simulated["use_mock_hardware"] == "true"
        assert simulated["litegrip_dry_run"] == "true"
        assert simulated["litegrip_hardware_enable"] == "false"

    # Hardware, both devices.
    both = hardware(False, True, True)
    assert both["use_mock_hardware"] == "false"
    assert both["litegrip_dry_run"] == "false"
    assert both["litegrip_hardware_enable"] == "true"

    # Hardware, the arm only: the gripper stays simulated and its gate stays shut.
    arm_only = hardware(False, True, False)
    assert arm_only["use_mock_hardware"] == "false"
    assert arm_only["litegrip_dry_run"] == "true"
    assert arm_only["litegrip_hardware_enable"] == "false"

    # Hardware, the gripper only: the arm is simulated, and only the gripper moves.
    gripper_only = hardware(False, False, True)
    assert gripper_only["use_mock_hardware"] == "true"
    assert gripper_only["litegrip_dry_run"] == "false"
    assert gripper_only["litegrip_hardware_enable"] == "true"


def test_the_gripper_gets_the_bound_it_needs_to_send_motion():
    """Without a positive worst-case velocity the component sends no motion at all.

    Its default is a fail-closed negative meaning "not given", and the component
    then refuses every motion frame while the motor is enabled and holding and the
    controller accepts its goals - a real gripper that is plainly on and never
    moves, next to a mock one that moves, because the mock is not that component.
    So the launch has to pass one, and it has to be positive.
    """
    value = float(_common().hardware_arguments(False, True, True)[
        "litegrip_max_feedback_velocity_rad_s"])
    assert value > 0.0, (
        "the launch passes litegrip_max_feedback_velocity_rad_s=%r; a non-positive value is "
        "the component's fail-closed \"not given\" and stops every motion frame" % value)


def test_the_controllers_spawned_are_the_ones_that_are_controlled():
    names = _common().controller_names
    assert names(True, False) == ["joint_state_broadcaster", "vr_forward_controller"]
    assert names(False, True) == ["joint_state_broadcaster", "gripper_controller"]
    assert names(True, True) == [
        "joint_state_broadcaster", "vr_forward_controller", "gripper_controller"]


#: Where each argument the launch emits is actually declared. The mode argument
#: belongs to this stack's own description - it is what chooses between the mock
#: hardware block and the real ones - and the gripper's two belong to its component.
#: Every emitted name has to appear here, so a new one cannot be added without
#: saying which description has to declare it.
_ARGUMENT_SOURCES = {
    "use_mock_hardware": ("litearm_vr_moveit_config", "urdf/litearm_vr.urdf.xacro"),
    "litegrip_dry_run": ("litegrip_ros2_control", "urdf/litegrip.ros2_control.xacro"),
    "litegrip_hardware_enable": ("litegrip_ros2_control", "urdf/litegrip.ros2_control.xacro"),
    "litegrip_max_feedback_velocity_rad_s": (
        "litegrip_ros2_control", "urdf/litegrip.ros2_control.xacro"),
}


def _xacro_arguments(package, relative):
    import xml.etree.ElementTree as ET

    path = os.path.join(get_package_share_directory(package), relative)
    with open(path, "r", encoding="utf-8") as stream:
        root = ET.fromstring(stream.read())
    return {
        element.get("name")
        for element in root.iter("{http://www.ros.org/wiki/xacro}arg")
    }


def test_the_component_arguments_are_the_ones_the_components_read():
    """A xacro argument nobody passes keeps its default, silently.

    So every name the launch emits is checked against the description that has to
    declare it: a spelling mistake here is a component configured by a default
    nobody chose, which looks like a component that works.
    """
    emitted = set(_common().hardware_arguments(False, True, True))
    assert emitted == set(_ARGUMENT_SOURCES), (
        "the launch emits %s; the test knows where %s of them live, so one was added "
        "without saying which description has to declare it"
        % (sorted(emitted), sorted(_ARGUMENT_SOURCES)))

    for name in sorted(emitted):
        package, relative = _ARGUMENT_SOURCES[name]
        assert name in _xacro_arguments(package, relative), (
            "%s is passed to %s, which does not declare it: the argument would be ignored "
            "and the description's default used instead" % (name, package))
