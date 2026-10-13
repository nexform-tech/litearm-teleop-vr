# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The description-mesh check says the right thing about a synthetic description.

The check exists because its failure mode has no error anywhere near its cause:
a description that names its meshes in a package the consumer cannot see produces
a stack that runs, an arm that moves and a display that draws nothing. What is
worth testing is therefore the message, not the running: which package it names,
and that it names it only when the package really is unreachable.
"""

import os
import subprocess
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOL = os.path.join(os.path.dirname(_HERE), "tools", "check_description_meshes.py")

_URDF = """<?xml version="1.0"?>
<robot name="probe">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://{package}/meshes/base_link.STL" />
      </geometry>
    </visual>
  </link>
</robot>
"""


def _run(arguments, prefix_path):
    environment = dict(os.environ)
    environment["AMENT_PREFIX_PATH"] = prefix_path
    return subprocess.run(
        [sys.executable, _TOOL] + arguments, capture_output=True, text=True, check=False,
        env=environment).stdout


def _description(tmp_path, package, name="probe.urdf"):
    directory = tmp_path / "share" / "probe_description" / "urdf"
    directory.mkdir(parents=True)
    (directory / name).write_text(_URDF.format(package=package), encoding="utf-8")
    return str(tmp_path / "share" / "probe_description")


def _register(tmp_path, package):
    """Make `package` look installed: the file the ament index keeps per package."""
    index = tmp_path / "install" / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / package).write_text("", encoding="utf-8")
    return str(tmp_path / "install")


def test_reports_a_reference_it_cannot_resolve(tmp_path):
    root = _description(tmp_path, "litearm")
    output = _run([root], prefix_path="/opt/ros/humble")
    assert "litearm" in output
    assert "cannot be resolved" in output


def test_stays_quiet_about_a_reference_that_resolves(tmp_path):
    root = _description(tmp_path, "litearm_description")
    prefix = _register(tmp_path, "litearm_description")
    output = _run([root], prefix_path=prefix)
    assert "cannot be resolved" not in output
    assert "all packages resolve" in output


def test_registration_is_what_counts_not_a_directory(tmp_path):
    """A share directory dropped into an install tree is not a package.

    This is the other half of the confusion the check is about: the meshes can be
    present as files and still not be reachable, because what resolves a
    ``package://`` reference is the index entry, not the directory.
    """
    root = _description(tmp_path, "litearm")
    (tmp_path / "install" / "share" / "litearm").mkdir(parents=True)
    output = _run([root], prefix_path=str(tmp_path / "install"))
    assert "cannot be resolved" in output


def test_a_missing_share_directory_is_said_rather_than_crashed(tmp_path):
    output = _run([str(tmp_path / "absent")], prefix_path="/opt/ros/humble")
    assert "does not exist" in output
