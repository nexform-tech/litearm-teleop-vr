# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Guards the two clocks and the joint order that the two nodes share.

The command path is three processes with three clocks in it, and only one of the
three relationships can be checked from this repository:

  * the headset, at about 72 Hz, which is deliberately NOT aligned to anything.
    Frames arrive when the link allows, and the node's job is to keep that timing
    out of the command stream rather than to follow it - the target advances on
    the node's tick, so the headset's rate decides only how fresh the pose in
    that step is. Nothing here needs to match it.

  * the teleoperation node, whose `control_rate_hz` is the master for both the
    command stream and the tick the target advances on. This is the relationship
    the first test below pins.

  * the controller_manager, whose `update_rate` is how often the controller reads
    the command and writes the hardware. It also has to match the firmware's
    state report rate - a faster loop than the report is a stair-stepped feedback
    signal, which the arm's own configuration documents at length. That third
    clock lives in another repository, so what is checkable here is only that the
    node and the controller agree; the reasoning is in `vr_controllers.yaml` and
    in the README.

The joint order is checked here as well, because it is the other half of the same
contract: the command is a flat array, and its meaning is the order of the names
the two sides were configured with. A mismatch moves the wrong axis, and it looks
like plausible data rather than like a fault.
"""

import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

_PACKAGE = "litearm_vr_teleop"


def _config(name):
    path = os.path.join(get_package_share_directory(_PACKAGE), "config", name)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _teleop():
    return _config("vr_teleop.yaml")["vr_teleop"]["ros__parameters"]


def _controllers():
    return _config("vr_controllers.yaml")


def test_the_node_publishes_at_the_controller_rate():
    """A stream at the controller's own rate is what the ceiling assumes.

    The controller reads the latest command once per cycle and holds it
    otherwise, so a node publishing SLOWER than the controller leaves the
    reference advancing in steps the size of one command, with nothing written in
    between. That is the staircase this stack has already been bitten by once,
    from the other direction, in the feedback path.

    Publishing faster is not a substitute: the extra commands are overwritten
    before the controller ever sees them, because the handover is a single slot
    and not a queue, which is deliberate - a queue would trade latency for
    smoothness and teleoperation cannot pay that.

    So the two rates are the same number, and this test is what keeps them the
    same number.
    """
    node_rate = float(_teleop()["control_rate_hz"])
    manager_rate = float(_controllers()["controller_manager"]["ros__parameters"]["update_rate"])
    assert node_rate == manager_rate, (
        "vr_teleop.yaml says control_rate_hz: {0} while vr_controllers.yaml says "
        "update_rate: {1}. The node produces the command stream and the controller "
        "consumes one command per cycle; set them to the same number.".format(
            node_rate, manager_rate))


def test_the_rates_are_at_least_the_headsets():
    """A tick slower than the headset would drop hand motion on the floor.

    The node's tick takes the newest frame and advances the target by the
    increment the frames so far describe, so a tick below the headset's rate
    means several frames are folded into one step. Nothing is lost - the
    increments accumulate - but the target is then updated less often than the
    operator's hand is measured, which is the lag this rate exists to bound.
    """
    node_rate = float(_teleop()["control_rate_hz"])
    headset_rate = 72.0  # the measured Quest 3 rate; see vr_teleop_node.cpp
    assert node_rate >= headset_rate, (
        "control_rate_hz is {0} but the headset publishes at about {1} Hz. The tick "
        "that advances the Cartesian target would then be slower than the pose it "
        "advances it from.".format(node_rate, headset_rate))


@pytest.mark.parametrize("controller", ["vr_forward_controller"])
def test_the_joint_order_matches_between_the_node_and_the_controller(controller):
    """One list of joint names, two files, and a flat array between them.

    The command carries no names: the controller writes value `i` to the joint it
    lists in position `i`, and the node built value `i` for the joint in its own
    list position `i`. The controller's own configuration says what happens when
    the two disagree - the arm moves, the data looks plausible, and only the
    operator notices. It has been true of the broadcaster's interface order
    before, which is why the comment there says to read `/joint_states` by name.
    """
    node_joints = list(_teleop()["joint_names"])
    controller_joints = list(_controllers()[controller]["ros__parameters"]["joints"])
    assert node_joints == controller_joints, (
        "vr_teleop.yaml lists joint_names {0} while vr_controllers.yaml lists {1} for "
        "{2}. The command topic carries values in the node's order and the controller "
        "writes them in its own, so a difference here moves the wrong axis.".format(
            node_joints, controller_joints, controller))
