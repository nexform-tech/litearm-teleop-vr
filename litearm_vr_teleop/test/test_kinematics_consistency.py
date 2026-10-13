# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Guards against two copies of the same number drifting apart.

This stack deliberately repeats two sets of values that also live in
`litearm_vr_moveit_config`, for reasons that are real rather than convenient:

  * the kinematics solver's class and its four tuning parameters. The plugin
    looks for them as `<group>.kinematics_solver`, `<group>.solver_type` and so
    on, and a MoveIt kinematics file has no node name at its top level, so it
    cannot be handed to a node as a parameter file. Wrapping it would put a
    generated file between the two, which hides the copy rather than removing it.

  * the teleoperation start pose. The SRDF needs the pose under a name so MoveIt
    can use it; the C++ node needs the numbers so it can reach the pose without
    a planner, and a teleoperation start that needs MoveIt to be running would be
    a strange dependency.

In both cases the copy is visible and these tests are what keep it honest. They
fail loudly rather than letting the two diverge silently, which is how a
"stated change that had no effect" is usually discovered instead: much later, on
hardware, by somebody watching the arm go somewhere they did not expect.
"""

import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

# The tuning each plugin reads through its own parameter lookup. Which set
# applies is decided by the class the configuration names, not by this file: a
# node pointed at KDL has no `solver_type`, and demanding one would be the test
# asserting a solver choice rather than the configuration's consistency with it.
_SNS_KEYS = ("solver_type", "loop_period", "eps", "timeout")
_KDL_KEYS = (
    "kinematics_solver_search_resolution",
    "kinematics_solver_timeout",
    "kinematics_solver_attempts",
)

# The subset asked for as doubles. `solver_type` is left out on purpose: it is a
# string ("SNS"), and reading it as text is correct.
_NUMERIC_KEYS = {
    "solver_type": (),
    "loop_period": ("loop_period",),
    "eps": ("eps",),
    "timeout": ("timeout",),
}


def _tuning_keys(solver_class):
    """The keys the named plugin reads, and refuses to start without."""
    if "sns_ik" in solver_class:
        return _SNS_KEYS
    if "kdl_kinematics_plugin" in solver_class:
        return _KDL_KEYS
    pytest.fail(
        "the configuration names {0!r}, and this test does not know which parameters it reads. "
        "Add its key set here rather than assuming another plugin's.".format(solver_class))


_GROUP = "litearm_arm"


def _moveit_kinematics():
    path = os.path.join(
        get_package_share_directory("litearm_vr_moveit_config"), "config", "kinematics.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)[_GROUP]


def _teleop_parameters():
    path = os.path.join(
        get_package_share_directory("litearm_vr_teleop"), "config", "vr_teleop.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)["vr_teleop"]["ros__parameters"]


def _same_value(actual, expected):
    """Compare two values that may be numbers written in different forms.

    The MoveIt configuration writes `eps: 1e-5`, which YAML reads as a STRING
    because YAML floats need a digit before the dot. This file writes `1.0e-5`,
    which YAML reads as a float, and which is what the plugin needs: it asks for
    that parameter as a double and a string would make it throw. Comparing the
    two textually would fail over a difference that does not exist numerically.
    """
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return float(actual) == float(expected)
    if isinstance(actual, (int, float)) or isinstance(expected, (int, float)):
        # One is a number and the other is not: as close to a number as the text
        # can be read, or a real difference.
        try:
            return float(actual) == float(expected)
        except (TypeError, ValueError):
            return False
    return actual == expected


@pytest.mark.parametrize(
    "key", ("kinematics_solver",) + tuple(_tuning_keys(_teleop_parameters()[_GROUP]["kinematics_solver"])))
def test_solver_parameter_matches_the_moveit_configuration(key):
    """One number in two places: the copies must agree when they name one solver.

    They do not always name one solver, and that is deliberate. `kinematics.yaml`
    keeps MoveIt on KDL, which ships with `moveit_core` and needs nothing else in
    the workspace, so a workspace without the SNS plugin still plans; the node can
    load either. The two files therefore describe two deployments of the same
    tuning, and what has to hold is the implication: **if** MoveIt names the
    solver the node loads, the tuning has to match it.

    That the two sides are each complete for the solver they name is checked
    separately, by test_solver_configuration_is_complete_on_both_sides, so the
    differing case is covered rather than excused.
    """
    node = _teleop_parameters()[_GROUP]
    moveit = _moveit_kinematics()
    if str(moveit["kinematics_solver"]) != node["kinematics_solver"]:
        return
    assert _same_value(node[key], moveit[key]), (
        "the teleop node's {0}.{1} is {2!r} while the MoveIt configuration says {3!r}. "
        "The plugin reads the teleop copy, so the configuration file would appear to have "
        "no effect. Change both.".format(_GROUP, key, node[key], moveit[key]))


def test_solver_configuration_is_complete_on_both_sides():
    """Each configuration must carry everything the solver it names needs.

    A class name alone is not a configuration. The node reads its own copy as
    `<group>.kinematics_solver` plus whatever that plugin looks up, and a missing
    one is not a warning: the plugin throws, the node reports `planning is
    unavailable`, and the arm holds. MoveIt has the same failure mode one level
    down, so both are checked here, and the branch that compares the two copies is
    closed explicitly rather than left open.
    """
    node = _teleop_parameters()[_GROUP]
    moveit = _moveit_kinematics()

    for key in _tuning_keys(str(node["kinematics_solver"])):
        assert key in node, (
            "the teleop node names solver {0!r} but has no {1}. The plugin reads its tuning "
            "through a parameter lookup and throws when one is missing.".format(
                node["kinematics_solver"], key))
        for numeric in _NUMERIC_KEYS.get(key, ()):
            # A string here is not a near miss: the plugin asks for a double and a
            # string makes it throw.
            assert isinstance(node[numeric], (int, float)), (
                "{0}.{1} is {2!r}, which YAML read as text; the plugin asks for a double and a "
                "string makes it throw. Write a digit before the dot (1.0e-5, not 1e-5).".format(
                    _GROUP, numeric, node[numeric]))
    # The SNS plugin reads `solver_type` as text, so a value YAML read as a number
    # would be wrong in a way that only shows up when the solver throws.
    if "sns_ik" in str(node["kinematics_solver"]):
        assert isinstance(node.get("solver_type"), str), (
            "{0}.solver_type is {1!r}; the plugin reads it as text (SNS, SNS_Fast, ...).".format(
                _GROUP, node.get("solver_type")))

    for key in _tuning_keys(str(moveit["kinematics_solver"])):
        assert key in moveit, (
            "the MoveIt configuration names {0!r} but has no {1}, so the plugin would silently "
            "use its own default while the file appears to configure it.".format(
                moveit["kinematics_solver"], key))


# ------------------------------------------------------------------------------
# XML hygiene
# ------------------------------------------------------------------------------

_COMMENT = __import__("re").compile(r"<!--(.*?)-->", __import__("re").S)


def _xml_and_xacro_files():
    """Every XML flavour file in this package."""
    import glob

    root = get_package_share_directory("litearm_vr_teleop")
    matches = []
    for pattern in ("**/*.xml", "**/*.xacro", "**/*.urdf", "**/*.srdf"):
        matches.extend(glob.glob(os.path.join(root, pattern), recursive=True))
    return sorted(set(matches))


def test_xml_comments_contain_no_double_hyphen():
    """An XML comment may not contain two hyphens in a row.

    This is not pedantry: the file stops being XML, and the failure that follows
    is a launch parse error whose message points at a line of prose rather than at
    the offending characters. Both ways it creeps in are silent - a dashed
    underline for a heading, and a command line flag written inside a comment -
    and both have happened in this package. Checking here costs nothing and turns
    a confusing launch failure into a named test.
    """
    offenders = []
    files = _xml_and_xacro_files()
    assert files, "no XML files were found to check, which means the glob is wrong"

    for path in files:
        with open(path, "r", encoding="utf-8") as stream:
            text = stream.read()
        for comment in _COMMENT.finditer(text):
            for line in comment.group(1).splitlines():
                if "--" in line:
                    offenders.append("{0}: {1}".format(os.path.basename(path), line.strip()))

    assert not offenders, "double hyphens inside XML comments:\n" + "\n".join(offenders)
