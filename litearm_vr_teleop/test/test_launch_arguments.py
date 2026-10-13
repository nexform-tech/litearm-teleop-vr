# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The launch files and the hardware component must agree about argument names.

The gripper's hardware is configured by xacro arguments, and a xacro argument
that nobody passes is not an error: it silently keeps its default. So a launch
file that spells `litegrip_can_id` as `litegrip_canid` produces a gripper that
runs on the wrong bus, or on can0 when the rig is on can1, with nothing anywhere
saying so - the same class of failure as the mesh package and the mapping frame,
and the same answer: make the two names checkable against each other.

The second thing pinned here is the safe direction of the defaults. A launch file
cannot know whether a gripper is attached, so `litegrip_hardware_enable` has to
default to false and `litegrip_dry_run` to true: a rig that has one says so, and
a rig that does not can never be moved by a command that was meant for a bench.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

_LAUNCH = "litearm_vr_teleop"

#: The gripper's hardware arguments, as the litegrip component's xacro names them.
_GRIPPER_ARGS = (
    "litegrip_dry_run",
    "litegrip_hardware_enable",
    "litegrip_channel",
    "litegrip_can_id",
    "litegrip_mst_id",
    "litegrip_max_feedback_velocity_rad_s",
)

#: Where each argument lives, and why:
#:   stack  declares them, so every entry point can pass them through;
#:   sim    runs the real hardware branch with a fake arm, which is the bench
#:          that has a real gripper and no arm - it has to accept all of them;
#:   virtual is pinned to the mock branch, which replaces the gripper's component
#:          along with the arm's, so it must NOT offer arguments that do nothing.
_STACK = "launch/teleop_stack.launch.xml"
_SIM = "launch/teleop_sim.launch.xml"
_VIRTUAL = "launch/teleop_virtual.launch.xml"


def _launch_path(name):
    return os.path.join(get_package_share_directory(_LAUNCH), name)


def _arguments(name):
    root = ET.parse(_launch_path(name)).getroot()
    return {element.get("name"): element.get("default") for element in root.iter("arg")}


def _forwarded(name):
    root = ET.parse(_launch_path(name)).getroot()
    return {
        element.get("name")
        for element in root.iter("arg")
        if element.get("value", "").startswith("$(var ")
    }


def _litegrip_xacro_arguments():
    path = os.path.join(
        get_package_share_directory("litegrip_ros2_control"), "urdf",
        "litegrip.ros2_control.xacro")
    with open(path, "r", encoding="utf-8") as stream:
        text = stream.read()
    root = ET.fromstring(text)
    return {
        element.get("name")
        for element in root.iter("{http://www.ros.org/wiki/xacro}arg")
    }


@pytest.mark.parametrize("name", _GRIPPER_ARGS)
def test_the_stack_declares_the_gripper_arguments(name):
    """Every one of them is a launch argument, not a value baked into the xacro.

    The bench decides what is attached; the xacro's job is to describe it.
    """
    assert name in _arguments(_STACK), (
        "teleop_stack.launch.xml does not declare '%s', so a rig cannot change it without "
        "editing a file" % name)


@pytest.mark.parametrize("name", _GRIPPER_ARGS)
def test_the_sim_entry_point_passes_them_through(name):
    """The fake-arm bench is where a real gripper is used, so it must accept them."""
    assert name in _arguments(_SIM), (
        "teleop_sim.launch.xml does not declare '%s': the entry point that runs a real "
        "gripper against a fake arm could not configure it" % name)
    assert name in _forwarded(_SIM), (
        "teleop_sim.launch.xml declares '%s' but does not pass it to teleop_stack, so "
        "the value would be ignored and the default would apply" % name)


@pytest.mark.parametrize("name", _GRIPPER_ARGS)
def test_the_names_are_the_ones_the_component_reads(name):
    """A xacro argument nobody passes keeps its default, silently.

    Which is why the names are checked rather than trusted: the failure is a
    gripper configured by a default nobody chose, and it looks like a gripper that
    works, until someone asks for the other bus.
    """
    declared = _litegrip_xacro_arguments()
    assert name in declared, (
        "teleop_stack.launch.xml passes '%s', but litegrip.ros2_control.xacro declares "
        "only %s. The argument would be ignored and the xacro's default used instead"
        % (name, ", ".join(sorted(declared))))


def test_the_defaults_are_the_safe_ones():
    """Nothing moves a motor unless a launch file says to.

    A launch file cannot know whether a gripper is attached, so both gates stay
    closed: dry run on, motor gate off. Opening them is an explicit act, and it is
    the pair that opens them - not either one alone.
    """
    arguments = _arguments(_STACK)
    assert arguments["litegrip_dry_run"] == "true"
    assert arguments["litegrip_hardware_enable"] == "false"


def test_the_mock_entry_point_offers_no_gripper_arguments():
    """It cannot drive a real gripper, so it must not look as though it could.

    The mock branch is one hardware component covering the arm and the gripper,
    so any gripper argument there would be accepted, ignored, and read as "the
    gripper is simulated because the mock is" - which is true, and is exactly the
    wrong reason to believe it.
    """
    offered = [name for name in _arguments(_VIRTUAL) if name.startswith("litegrip_")]
    assert offered == [], (
        "teleop_virtual.launch.xml offers %s, but its mock branch replaces the gripper's "
        "component: the values would do nothing" % offered)
