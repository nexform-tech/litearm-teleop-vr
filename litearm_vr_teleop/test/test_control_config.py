# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The control logic's configuration: two of them, one switch, one file.

What is worth pinning here is that the switch is a switch - a value the node
accepts, not a spelling that silently falls back to the default - and that each
logic's calibration exists, because the two read the same three numbers in
different frames and a missing one would be a mirror nobody chose. The retired
keys are pinned as absent for the same reason: a file that still says
`rotation_mode` would be ignored in silence, and silence is what this package
keeps having to fix.
"""

import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

_PACKAGE = "litearm_vr_teleop"
_CALIBRATIONS = ("base_link_rotation_signs", "tool_rotation_signs")
_RETIRED = ("rotation_mode", "rotation_signs")


def _parameters():
    path = os.path.join(
        get_package_share_directory(_PACKAGE), "config", "vr_teleop.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)["vr_teleop"]["ros__parameters"]


def test_the_switch_names_one_of_the_two_logics():
    frame = _parameters()["control_frame"]
    assert frame in ("base_link", "tool"), (
        "control_frame is %r; the node accepts 'base_link' and 'tool' and would fall back to "
        "base_link for anything else, which is a silent way to lose a switch" % frame)


@pytest.mark.parametrize("name", _CALIBRATIONS)
def test_every_logic_carries_its_own_calibration(name):
    signs = _parameters()[name]
    assert isinstance(signs, list) and len(signs) == 3, (
        "%s is %r; each control logic needs its own three numbers" % (name, signs))
    assert all(float(value) in (-1.0, 1.0) for value in signs), (
        "%s is %r; a calibration mirrors axes, so every component is a sign" % (name, signs))


def test_the_tcp_offset_lives_in_the_same_file():
    """It used to be its own file, loaded beside this one.

    Keeping it here is the same statement the launch files make about their
    arguments: what the tool is, and where the tool is, are properties of the rig,
    and a property of the rig belongs in one file.
    """
    offset = _parameters()["tcp_offset"]
    assert isinstance(offset, list) and len(offset) == 3, (
        "tcp_offset is %r; it is the tip to tool vector in the tip frame, in metres"
        % (offset,))


@pytest.mark.parametrize("name", _RETIRED)
def test_the_retired_keys_are_gone(name):
    assert name not in _parameters(), (
        "vr_teleop.yaml still sets '%s'. The node reads 'control_frame' and the two per logic "
        "calibrations, so this value would do nothing at all" % name)
