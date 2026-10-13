#!/usr/bin/env python3
# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Check that every package a description's meshes live in can be found here.

Why this exists
---------------
A robot description names its meshes as ``package://<package>/meshes/x.STL``, and
only the environment the consumer runs in can resolve that. So a description can
be perfectly valid on the machine that composed it and unusable on the machine
that displays it, which is a failure with no error anywhere near its cause: the
stack comes up, the joint states arrive, the arm moves when commanded, and the
only symptom is rviz drawing nothing while it complains about a package that
does exist - somewhere else. The one that bit this rig: the arm description moved
from a package called ``litearm`` to one called ``litearm_description``, and every
rviz started without the workspace that still provides the old name kept a
latch of the old description and reported it.

What it checks
--------------
Every ``package://`` reference in the description files it is pointed at, against
the AMENT prefix path of THIS process. The registration test is the file the
ament index keeps per package, so no ROS python is needed and a share directory
copied into an install tree does not count as a package - which is correct, and
is the other half of the same confusion.

What it does not do
-------------------
It does not fail. A description whose meshes cannot be resolved still teleoperates
the arm, and refusing to start would break a working configuration to complain
about a display; the point is to say it out loud, in one line, before rviz does it
worse.

Usage (the launch file runs this):
    check_description_meshes.py <share directory> [<share directory> ...]
"""

import os
import re
import sys

#: What a mesh reference looks like inside a description file. The package name
#: is the only part that can fail to resolve, so it is the only part captured.
_REFERENCE = re.compile(r"package://([A-Za-z0-9_]+)/")

#: Description files, by suffix. The same reference appears in a URDF, in a xacro
#: that generates one, and in the SRDF's group states.
_SUFFIXES = (".urdf", ".xacro", ".srdf")


def registered_prefix(name, prefixes):
    """The prefix of the install tree that has `name` registered, or None."""
    for prefix in prefixes:
        index = os.path.join(prefix, "share", "ament_index", "resource_index", "packages", name)
        if os.path.exists(index):
            return prefix
    return None


def references(root):
    """Every package name a description file under `root` refers to, with a count."""
    found = {}
    for directory, _dirs, files in os.walk(root):
        for name in files:
            if not name.endswith(_SUFFIXES):
                continue
            path = os.path.join(directory, name)
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as stream:
                    text = stream.read()
            except OSError:
                continue
            for package in _REFERENCE.findall(text):
                found[package] = found.get(package, 0) + 1
    return found


def main(argv):
    roots = argv[1:]
    if not roots:
        print("check_description_meshes.py: no share directory given", file=sys.stderr)
        return 2

    prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    seen = {}
    for root in roots:
        if not os.path.isdir(root):
            print("description meshes: %s does not exist" % root)
            continue
        for package, count in references(root).items():
            seen[package] = seen.get(package, 0) + count

    if not seen:
        print("description meshes: no package:// references found in %s" % ", ".join(roots))
        return 0

    missing = {p: c for p, c in seen.items() if registered_prefix(p, prefixes) is None}
    listed = ", ".join("%s x%d" % (p, seen[p]) for p in sorted(seen))
    if not missing:
        print("description meshes: all packages resolve (%s)" % listed)
        return 0

    print(
        "description meshes: %s cannot be resolved against the %d install prefix(es) on "
        "AMENT_PREFIX_PATH (%s). Source the workspace that provides %s before launching, or "
        "the arm will teleoperate and draw nothing." % (
            ", ".join(sorted(missing)), len(prefixes), listed, " and ".join(sorted(missing))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
