#!/usr/bin/env python3
# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Drive the teleoperation stack from a recorded episode or from a synthetic hand.

This is the harness half of a measurement. The two entry points that can be
driven by a headset are also drivable by anything that speaks the link's wire
format, and that is what this script does: it connects to
``ws://127.0.0.1:<port>`` as if it were the headset page and streams controller
frames. Everything downstream is the real stack - the link, the frame parsing,
the grip edge, the increment mapper, the inverse kinematics, the joint filter,
the forward controller, the firmware's window into all of it. Nothing about the
robot is simulated here.

Two sources
-----------
``--synth``
    A still hand, optionally with white noise (``--noise-mm``). This is the
    minimal reproducible case: the operator is not moving, so every bit of
    motion in the command stream is the chain's own. Use it as the noise floor.

``--dataset DIR --episode N``
    A recorded teleoperation episode, replayed through its own arm motion. The
    columns are ``observation.state`` and ``action`` (8 values: 7 joints and the
    gripper) at 30 fps. The recorded joints are the commanded joints, so they are
    run through forward kinematics to recover the tool path, and the inverse of
    the mapper's correspondence turns that path back into a hand path. The stack
    then turns the hand path back into a tool path; the round trip is asserted by
    ``--selftest`` to 1e-9, with no ROS and no socket involved.

Why the hand path is not simply ``M * (p_tool - p_tool(0))``
-----------------------------------------------------------
The mapper (``TeleopIncrementMapper::feed``) reads the controller's *relative*
motion: it forms ``delta = C_prev^-1 * C_cur`` in the controller's own frame, so
the translation delta arrives rotated by the inverse of the controller's attitude
before ``M^T`` carries it into the robot frame:

    dp_tool = M^T * R_hand(prev)^T * dp_hand

Inverting that gives the two relations this script uses, with
``M = translation_remap`` (x right, y up, z back -> x right, y forward, z up):

    Y(t)     = R_hand(t) rotated into the robot frame = M^T * R_c(t) * M
    Y(t)     = R_tool(t) * R_tool(0)^T                     (kWorld increment)
    p_hand(t) = p_hand(t-1) + R_c(t-1) * M * dp_tool(t)

The first two lines are the prompt's ``hand_R_world(t) = M * R_tool(t) *
R_tool(0)^T * M^T`` written for the frame the mapper actually reads (``R_c`` is
that value, because ``R_c(0) = I``). The third line carries the previous
attitude, which is why the plain ``p_hand0 + M * dp_tool`` composition - correct
only while the hand's attitude is identity - is measurably wrong on real data
(4.6 cm on episode 0 of cup_grasp_v4). ``--selftest`` prints both residuals.

Timing
------
Frames are paced by ``1/rate + U(-jitter, +jitter)`` per frame, never by a fixed
period: the headset is jittery and the harness has to be too, or it would measure
its own lack of jitter. The realisation is reported at the end (min/mean/max
interval, achieved rate).

Which interpreter runs this
---------------------------
``numpy`` and ``websockets`` are not in the system Python; the dataset also needs
``pyarrow``. ``$HOME/venvs/litearm-lerobot/bin/python`` has all three, so the
script re-executes itself into it (chosen by ``LITEARM_REPLAY_PYTHON``) unless
the current interpreter already has what is needed. The modules are imported on
first use rather than at the top of the file, because the re-exec has to happen
before anything needs them.

Usage
-----
    python3 headset_replay.py --synth --duration 20
    python3 headset_replay.py --dataset ~/bak/data/cup_grasp_v4 --episode 0 --loop --duration 20
    python3 headset_replay.py --selftest
    python3 headset_replay.py --synth --dry-run --duration 1
"""

import argparse
import asyncio
import json
import math
import os
import random
import sys
import time

# Guard against re-executing forever if the chosen interpreter also lacks the
# modules. The value is opaque on purpose: it only has to exist.
_BOOTSTRAP_ENV = "LITEARM_REPLAY_BOOTSTRAPPED"
_DEFAULT_PYTHON = os.path.join(
    os.path.expanduser("~"), "venvs", "litearm-lerobot", "bin", "python")

# The measured correspondence of singleArmUprightMapping() in src/teleop_math.cpp:
# controller (x right, y up, z back) -> arm base (x right, y forward, z up).
TRANSLATION_REMAP = (
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
)

# The bare arm description, installed by litearm_description. Resolved without
# ROS so that --selftest can run in a plain interpreter (no rclpy, no socket).
_URDF_RELATIVE = os.path.join("share", "litearm_description", "urdf", "litearm_description.urdf")
_URDF_ROOTS = (
    os.path.join(os.path.expanduser("~"), "github", "install", "litearm_description"),
    os.path.join(os.path.expanduser("~"), "wkspace", "ros2_ws", "install", "litearm"),
)

# The chain the teleoperation node solves for: 7 revolute joints and the tool.
_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
_ROOT_LINK = "base_link"
_TIP_LINK = "ee_link"

_NUMPY = None


def _np():
    """numpy, imported on first use: the bootstrap has to run before it."""
    global _NUMPY
    if _NUMPY is None:
        import numpy
        _NUMPY = numpy
    return _NUMPY


# ------------------------------------------------------------------------------
# Interpreter bootstrap
# ------------------------------------------------------------------------------


def _missing(need_dataset):
    """The modules this run needs and this interpreter does not have."""
    wanted = ["numpy", "websockets"]
    if need_dataset:
        wanted.append("pyarrow")
    missing = []
    for name in wanted:
        try:
            __import__(name)
        except ImportError:
            missing.append(name)
    return missing


def bootstrap_interpreter(need_dataset):
    """Re-exec into an interpreter that has the modules this run needs."""
    if os.environ.get(_BOOTSTRAP_ENV) == "1":
        return
    missing = _missing(need_dataset)
    if not missing:
        return

    candidate = os.environ.get("LITEARM_REPLAY_PYTHON") or _DEFAULT_PYTHON
    if not os.path.isfile(candidate):
        sys.stderr.write(
            "headset_replay: this interpreter ({0}) has no {1}, and the fallback "
            "interpreter {2} does not exist.\nSet LITEARM_REPLAY_PYTHON to an interpreter "
            "that has them.\n".format(sys.executable, ", ".join(missing), candidate))
        raise SystemExit(2)

    environment = dict(os.environ, **{_BOOTSTRAP_ENV: "1"})
    sys.stderr.write(
        "headset_replay: re-executing into {0} (no {1} in {2})\n".format(
            candidate, ", ".join(missing), sys.executable))
    os.execve(candidate, [candidate, os.path.abspath(__file__)] + sys.argv[1:], environment)


# ------------------------------------------------------------------------------
# Forward kinematics straight off the URDF
# ------------------------------------------------------------------------------


def find_urdf(explicit=None):
    """The bare arm description, found without ROS.

    The task this came from resolves it with ``ros2 pkg prefix litearm_description``.
    A subprocess to the ROS CLI would work here too, but it would make the
    self-test depend on a sourced ROS environment for no benefit: the file lives
    at a fixed place inside the package prefix, so the prefixes are searched
    directly.
    """
    if explicit:
        if not os.path.isfile(explicit):
            raise SystemExit("no URDF at {0}".format(explicit))
        return explicit

    candidates = [_URDF_ROOTS[0]]
    for entry in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if entry:
            candidates.append(entry)
    candidates.extend(_URDF_ROOTS[1:])
    for root in candidates:
        for relative in (_URDF_RELATIVE, os.path.join("urdf", "litearm_description.urdf")):
            path = os.path.join(root, relative)
            if os.path.isfile(path):
                return path
    raise SystemExit(
        "could not find the arm description; pass --urdf <path> "
        "(looked for {0} under {1})".format(_URDF_RELATIVE, ", ".join(candidates)))


def _rpy_to_matrix(rpy):
    np = _np()
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _axis_angle_to_matrix(axis, angle):
    """Rodrigues, with the degenerate axis handled rather than trusted."""
    np = _np()
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return np.eye(3)
    unit = vector / norm
    skew = np.array([
        [0.0, -unit[2], unit[1]],
        [unit[2], 0.0, -unit[0]],
        [-unit[1], unit[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def load_chain(urdf_path, root=_ROOT_LINK, tip=_TIP_LINK):
    """(joints, links) pairs from the root link to the tip link, in order.

    Plain XML and no third-party URDF parser: the file is 7 revolute joints in a
    chain, each with an ``<origin xyz rpy>`` and an ``<axis xyz>``, and adding a
    dependency to read that would be the larger risk.
    """
    np = _np()
    import xml.etree.ElementTree as ET

    robot = ET.parse(urdf_path).getroot()
    by_child = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        axis = joint.find("axis")
        by_child[child.get("link")] = {
            "name": joint.get("name"),
            "type": joint.get("type"),
            "xyz": np.array([float(v) for v in (origin.get("xyz") if origin is not None
                                                else "0 0 0").split()]),
            "rpy": [float(v) for v in (origin.get("rpy") if origin is not None
                                       else "0 0 0").split()],
            "axis": (np.array([float(v) for v in axis.get("xyz").split()])
                     if axis is not None else np.array([0.0, 0.0, 1.0])),
            "parent": parent.get("link"),
        }

    chain = []
    link = tip
    while link != root:
        if link not in by_child:
            raise SystemExit(
                "the URDF has no chain from {0} to {1}: {2} has no parent joint".format(
                    root, tip, link))
        joint = by_child[link]
        chain.append(joint)
        link = joint["parent"]
    chain.reverse()

    movable = [joint for joint in chain if joint["type"] in ("revolute", "continuous",
                                                             "prismatic")]
    return chain, movable


def forward_kinematics(chain, positions, tip_to_root=True):
    """Tool pose from joints: (R (3,3), p (3,)) of the chain's last link.

    ``positions`` holds one angle per movable joint in chain order, which is the
    order of ``joint1..joint7`` in this URDF. The tip's fixed joint is folded in,
    so the result is the pose of the tool (``ee_link``), which is the frame the
    node's TCP offset is measured from.
    """
    np = _np()
    transform = np.eye(4)
    index = 0
    for joint in chain:
        origin = np.eye(4)
        origin[:3, :3] = _rpy_to_matrix(joint["rpy"])
        origin[:3, 3] = joint["xyz"]
        transform = transform @ origin
        motion = np.eye(4)
        if joint["type"] in ("revolute", "continuous"):
            motion[:3, :3] = _axis_angle_to_matrix(joint["axis"], float(positions[index]))
            index += 1
        elif joint["type"] == "prismatic":
            motion[:3, 3] = joint["axis"] / np.linalg.norm(joint["axis"]) * float(positions[index])
            index += 1
        transform = transform @ motion
    return transform[:3, :3].copy(), transform[:3, 3].copy()


# ------------------------------------------------------------------------------
# Rotations
# ------------------------------------------------------------------------------


def quaternion_from_matrix(matrix):
    """(x, y, z, w) of a rotation matrix, by the largest-diagonal branch.

    One branch never covers the whole sphere: the classic trace formula divides
    by a value that goes to zero at 180 degrees, which is exactly where a wrists
    wobble. The branch is chosen by the largest of the four candidate
    denominators instead, so the divisor is at least 0.5.
    """
    np = _np()
    m = np.asarray(matrix, dtype=float)
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / scale
        x = 0.25 * scale
        y = (m[0, 1] + m[1, 0]) / scale
        z = (m[0, 2] + m[2, 0]) / scale
    elif m[1, 1] > m[2, 2]:
        scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / scale
        x = (m[0, 1] + m[1, 0]) / scale
        y = 0.25 * scale
        z = (m[1, 2] + m[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / scale
        x = (m[0, 2] + m[2, 0]) / scale
        y = (m[1, 2] + m[2, 1]) / scale
        z = 0.25 * scale
    vector = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return vector / norm


def rotation_angle(matrix):
    """The rotation angle of a matrix, in radians (0 for a non-rotation)."""
    np = _np()
    cosine = (float(np.trace(np.asarray(matrix, dtype=float))) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, cosine)))


def rotation_residual(left, right):
    """Largest element-wise difference between two rotation matrices.

    Not the angle between them: ``acos`` of the trace takes the square root of
    the deviation, so it lifts float round-off (1e-16 in an entry) into 1e-8 rad,
    and any non-orthogonality drift the matrix products accumulate is read as a
    rotation that is not there. This metric is linear in the error, which is what
    a residual is supposed to be.
    """
    np = _np()
    return float(np.abs(np.asarray(left, dtype=float) - np.asarray(right, dtype=float)).max())


# ------------------------------------------------------------------------------
# The mapper's math, re-implemented
# ------------------------------------------------------------------------------


def mapper_increment(previous_position, previous_rotation, position, rotation, remap):
    """One frame pair through TeleopIncrementMapper::feed, kWorld mode.

    Returns (relative translation in the robot frame, relative rotation in the
    robot frame). The moving average is not modelled: one frame pair in, one
    increment out, which is what a window of one sample is.
    """
    np = _np()
    delta_translation = previous_rotation.T @ (position - previous_position)
    delta_rotation = previous_rotation.T @ rotation

    relative_translation = remap.T @ delta_translation
    previous_in_robot = remap.T @ previous_rotation
    relative_rotation = previous_in_robot @ delta_rotation @ previous_in_robot.T
    return relative_translation, relative_rotation


def hand_from_tool(tool_position, tool_rotation, remap, passes=1):
    """The hand path that makes the mapper reproduce the recorded tool path.

    The increments are what the mapper reads, so the path is built by
    accumulating them rather than by an absolute formula: that is also what
    makes ``passes > 1`` continuous, since a repeated episode then continues
    from where the previous one left the hand instead of teleporting back.
    """
    np = _np()
    remap = np.asarray(remap, dtype=float)
    count = len(tool_position)
    hand_position = np.zeros((count * passes, 3), dtype=float)
    hand_rotation = np.zeros((count * passes, 3, 3), dtype=float)

    # The hand's attitude carried into the robot frame, which is what the mapper
    # conjugates with: each recorded world increment multiplies it on the left.
    carried = np.eye(3)
    hand_rotation_current = remap @ carried @ remap.T
    hand_position_current = np.zeros(3, dtype=float)

    index = 0
    hand_position[index] = hand_position_current
    hand_rotation[index] = hand_rotation_current
    index += 1

    for _ in range(passes):
        for step in range(1, count):
            world_increment = tool_rotation[step] @ tool_rotation[step - 1].T
            carried = world_increment @ carried
            # The translation delta is rotated by the ATTITUDE THE HAND HAD
            # BEFORE THIS FRAME: that is what previous_controller_ does in feed().
            delta = hand_rotation_current @ remap @ (tool_position[step] - tool_position[step - 1])
            hand_position_current = hand_position_current + delta
            hand_rotation_current = remap @ carried @ remap.T

            hand_position[index] = hand_position_current
            hand_rotation[index] = hand_rotation_current
            index += 1
    return hand_position, hand_rotation


def tool_from_hand(hand_position, hand_rotation, start_position, start_rotation, remap):
    """The inverse direction, used by the self-test to close the loop."""
    np = _np()
    tool_position = np.zeros_like(hand_position)
    tool_rotation = np.zeros_like(hand_rotation)
    tool_position[0] = start_position
    tool_rotation[0] = start_rotation
    for index in range(1, len(hand_position)):
        relative_translation, relative_rotation = mapper_increment(
            hand_position[index - 1], hand_rotation[index - 1],
            hand_position[index], hand_rotation[index], remap)
        tool_position[index] = tool_position[index - 1] + relative_translation
        tool_rotation[index] = relative_rotation @ tool_rotation[index - 1]
    return tool_position, tool_rotation


# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------


def read_episode(dataset, episode, columns=("action", "timestamp")):
    """(seconds, (N, 8) joints) for one recorded episode.

    PyArrow directly rather than the lerobot reader: this is the file layout, and
    a reader that finds the file is not a reader that would notice the schema
    moving underneath it. The columns used are named explicitly for that reason.
    """
    np = _np()
    import pyarrow.parquet as parquet

    directory = os.path.join(dataset, "data")
    files = []
    for root, _, names in os.walk(directory):
        files.extend(sorted(os.path.join(root, name) for name in names
                            if name.endswith(".parquet")))
    if not files:
        raise SystemExit("no parquet files under {0}".format(directory))

    timestamps = []
    rows = []
    for path in files:
        table = parquet.read_table(path, columns=["episode_index", "frame_index"] + list(columns))
        index = np.asarray(table["episode_index"])
        selected = index == episode
        if not selected.any():
            continue
        frame = np.asarray(table["frame_index"])[selected]
        order = np.argsort(frame, kind="stable")
        rows.append(np.asarray(table[columns[0]].to_pylist(), dtype=float)[selected][order])
        timestamps.append(np.asarray(table[columns[1]].to_pylist(), dtype=float)[selected][order])

    if not rows:
        available = set()
        for path in files:
            table = parquet.read_table(path, columns=["episode_index"])
            available.update(int(value) for value in set(np.asarray(table["episode_index"]).tolist()))
        raise SystemExit("episode {0} is not in {1} (episodes present: {2})".format(
            episode, dataset, sorted(available)))

    seconds = np.concatenate(timestamps)
    joints = np.vstack(rows)
    seconds = seconds - seconds[0]
    return seconds, joints


def resample(seconds, values, rate):
    """Linear interpolation onto a uniform grid, all columns at once.

    The recorded trajectory is 30 fps; the link is a 72 Hz stream. Interpolating
    rather than repeating samples matters for what is being measured: a zero
    order hold would put a step into the hand path at every 33 ms boundary, and
    the whole question is what the command stream does with the hand's motion.
    """
    np = _np()
    if len(seconds) < 2:
        raise SystemExit("the episode has fewer than two samples")
    period = 1.0 / float(rate)
    count = int(math.floor((seconds[-1] - seconds[0]) / period)) + 1
    grid = seconds[0] + np.arange(count) * period
    out = np.empty((count, values.shape[1]), dtype=float)
    for column in range(values.shape[1]):
        out[:, column] = np.interp(grid, seconds, values[:, column])
    return grid, out


def tool_path_from_joints(joints, chain):
    """Tool poses for every row of an (N, 8) joint array; column 7 is the gripper."""
    np = _np()
    count = len(joints)
    position = np.zeros((count, 3), dtype=float)
    rotation = np.zeros((count, 3, 3), dtype=float)
    for index in range(count):
        rotation[index], position[index] = forward_kinematics(chain, joints[index][:7])
    return position, rotation


# ------------------------------------------------------------------------------
# Frames
# ------------------------------------------------------------------------------


class Frame(object):
    """One WebSocket message, in the shape headset_link.py documents."""

    __slots__ = ("position", "quaternion", "grip", "trigger")

    def __init__(self, position, quaternion, grip=False, trigger=0.0):
        self.position = position
        self.quaternion = quaternion
        self.grip = grip
        self.trigger = trigger

    def message(self, timestamp_ms):
        return {
            "timestamp": timestamp_ms,
            "rightController": {
                "position": {"x": float(self.position[0]), "y": float(self.position[1]),
                             "z": float(self.position[2])},
                "quaternion": {"x": float(self.quaternion[0]), "y": float(self.quaternion[1]),
                               "z": float(self.quaternion[2]), "w": float(self.quaternion[3])},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "gripActive": bool(self.grip),
                "trigger": float(self.trigger),
                "tracked": True,
            },
        }


def build_frames(args):
    """The whole stream, ready to send. Returns (frames, description)."""
    np = _np()
    remap = np.array(TRANSLATION_REMAP)

    if args.synth:
        # A still hand. Only the increments matter downstream, so the absolute
        # pose is irrelevant and zero is the honest one: nothing about the
        # recorded episode enters this case.
        count = int(round(float(args.duration) * args.rate)) + 1
        position = np.zeros((count, 3), dtype=float)
        if args.noise_mm > 0.0:
            position = position + np.random.normal(
                0.0, float(args.noise_mm) / 1000.0, size=(count, 3))
        quaternion = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (count, 1))
        description = "synthetic still hand, noise {0:.2f} mm".format(args.noise_mm)
        positions = position
        quaternions = quaternion
    else:
        chain, movable = load_chain(find_urdf(args.urdf))
        if len(movable) != 7:
            raise SystemExit(
                "the URDF chain {0} -> {1} has {2} movable joints, expected 7".format(
                    _ROOT_LINK, _TIP_LINK, len(movable)))
        seconds, joints = read_episode(args.dataset, args.episode)
        grid, resampled = resample(seconds, joints, args.rate)
        tool_position, tool_rotation = tool_path_from_joints(resampled, chain)

        passes = int(args.passes)
        if args.loop and args.duration is not None:
            # A pass is one episode's worth of resampled frames; enough passes to
            # cover the requested duration, plus the remainder.
            pass_seconds = len(grid) / float(args.rate)
            passes = max(1, int(math.ceil(float(args.duration) / pass_seconds)))
        hand_position, hand_rotation = hand_from_tool(
            tool_position, tool_rotation, remap, passes=passes)

        if not args.loop:
            wanted = len(grid)
            if args.duration is not None:
                wanted = min(wanted, int(round(float(args.duration) * args.rate)) + 1)
            hand_position = hand_position[:wanted]
            hand_rotation = hand_rotation[:wanted]
        elif args.duration is not None:
            wanted = min(len(hand_position), int(round(float(args.duration) * args.rate)) + 1)
            hand_position = hand_position[:wanted]
            hand_rotation = hand_rotation[:wanted]

        positions = hand_position
        quaternions = np.array([quaternion_from_matrix(matrix) for matrix in hand_rotation])
        description = (
            "episode {0} of {1}: {2} recorded frames at 30 fps ({3:.2f} s), resampled to "
            "{4} frames at {5:.0f} Hz, {6} pass(es)".format(
                args.episode, args.dataset, len(seconds), seconds[-1], len(positions),
                args.rate, passes))

    frames = []
    for index in range(len(positions)):
        frames.append(Frame(positions[index], quaternions[index]))
    return frames, description


def schedule(frames, args):
    """Grip and trigger on the stream's own clock."""
    count = len(frames)
    period = 1.0 / float(args.rate)
    for index, frame in enumerate(frames):
        seconds = index * period
        frame.grip = seconds >= float(args.grip_at)
        if args.trigger_close_at > 0.0:
            frame.trigger = 1.0 if seconds >= count * period - float(args.trigger_close_at) else 0.0
    return frames


# ------------------------------------------------------------------------------
# Sending
# ------------------------------------------------------------------------------


async def stream(frames, args):
    try:
        from websockets.asyncio.client import connect
    except ImportError:  # websockets < 13
        from websockets import connect

    uri = "ws://127.0.0.1:{0}".format(args.port)
    period = 1.0 / float(args.rate)
    started = time.monotonic()
    intervals = []
    previous = started

    async with connect(uri) as socket:
        print("connected to {0}; sending {1} frames at {2:.0f} Hz, jitter +/-{3:.1f} ms".format(
            uri, len(frames), args.rate, args.jitter * 1000.0), flush=True)
        next_at = time.monotonic()
        for index, frame in enumerate(frames):
            now = time.monotonic()
            message = frame.message(int((now - started) * 1000.0))
            await socket.send(json.dumps(message))
            intervals.append(now - previous)
            previous = now
            if index % max(1, int(args.rate * 2)) == 0:
                print("  t={0:5.2f}s frame {1}/{2} pos=({3:+.4f},{4:+.4f},{5:+.4f}) grip={6}".format(
                    now - started, index, len(frames), frame.position[0], frame.position[1],
                    frame.position[2], frame.grip), flush=True)
            # Never a fixed period: the interval carries the jitter, which is the
            # input this harness exists to inject.
            next_at += period + random.uniform(-args.jitter, args.jitter)
            delay = next_at - time.monotonic()
            if delay > 0.0:
                await asyncio.sleep(delay)
    return started, intervals


def report_timing(frames, intervals, started, finished):
    np = _np()
    elapsed = finished - started
    print("")
    print("frames sent      : {0}".format(len(frames)))
    print("elapsed          : {0:.3f} s".format(elapsed))
    print("achieved rate    : {0:.2f} Hz".format(len(frames) / elapsed if elapsed > 0 else 0.0))
    if intervals:
        array = np.asarray(intervals, dtype=float)[1:] * 1000.0
        if array.size:
            print("interval ms      : min {0:.3f}  mean {1:.3f}  max {2:.3f}  std {3:.3f}".format(
                array.min(), array.mean(), array.max(), array.std()))


def dry_run(frames, args):
    """Print what would be sent, and the summary a plot would need."""
    print("dry run: no socket is opened")
    print("frames   : {0}".format(len(frames)))
    for index in (0, min(1, len(frames) - 1), len(frames) - 1):
        print("  frame {0}: {1}".format(index, json.dumps(frames[index].message(0))))
    grip_frames = sum(1 for frame in frames if frame.grip)
    print("grip held: {0}/{1} frames ({2:.1f}%)".format(
        grip_frames, len(frames), 100.0 * grip_frames / max(1, len(frames))))
    return 0


# ------------------------------------------------------------------------------
# Self-test: no ROS, no socket
# ------------------------------------------------------------------------------


def _random_rotation(rng):
    np = _np()
    axis = rng.normal(size=3)
    axis = axis / np.linalg.norm(axis)
    angle = float(rng.uniform(-1.0, 1.0))
    return _axis_angle_to_matrix(axis, angle)


def selftest(args):
    """Prove the hand composition against the mapper, on data and on noise."""
    np = _np()
    remap = np.array(TRANSLATION_REMAP)
    failures = []

    print("── the correspondence matrix ──")
    identity_error = float(np.abs(remap.T @ remap - np.eye(3)).max())
    determinant = float(np.linalg.det(remap))
    print("  M^T M - I : {0:.3e}    det M : {1:.6f}".format(identity_error, determinant))
    if identity_error > 1e-12 or abs(determinant - 1.0) > 1e-12:
        failures.append("translation_remap is not orthonormal")

    print("")
    print("── quaternion conversion ──")
    rng = np.random.RandomState(20261012)
    worst_quaternion = 0.0
    for _ in range(2000):
        rotation = _random_rotation(rng)
        x, y, z, w = (float(value) for value in quaternion_from_matrix(rotation))
        recovered = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        worst_quaternion = max(worst_quaternion, rotation_residual(recovered, rotation))
    print("  worst element-wise error over 2000 random attitudes : {0:.3e}".format(
        worst_quaternion))
    if worst_quaternion > 1e-12:
        failures.append("quaternion conversion loses more than 1e-12")

    print("")
    print("── the mapper's inverse, on a random tool path ──")
    count = 400
    tool_position = np.cumsum(rng.normal(0.0, 0.004, size=(count, 3)), axis=0)
    tool_rotation = np.zeros((count, 3, 3))
    tool_rotation[0] = _random_rotation(rng)
    for index in range(1, count):
        tool_rotation[index] = _random_rotation(rng) @ tool_rotation[index - 1]
    hand_position, hand_rotation = hand_from_tool(tool_position, tool_rotation, remap)
    recovered_position, recovered_rotation = tool_from_hand(
        hand_position, hand_rotation, tool_position[0], tool_rotation[0], remap)
    position_error = float(np.abs(recovered_position - tool_position).max())
    rotation_error = max(rotation_residual(recovered_rotation[index], tool_rotation[index])
                         for index in range(count))
    print("  max |p_recovered - p_tool| : {0:.3e} m".format(position_error))
    print("  max |R_recovered - R_tool| : {0:.3e}".format(rotation_error))
    if position_error > 1e-9 or rotation_error > 1e-9:
        failures.append("the random-path round trip is not exact")
    np.save("/tmp/harness/selftest_hand_path.npy", hand_position)

    print("")
    print("── the recorded episode ──")
    chain, movable = load_chain(find_urdf(args.urdf))
    print("  urdf   : {0}".format(find_urdf(args.urdf)))
    print("  chain  : {0}".format(" -> ".join(joint["name"] for joint in chain)))
    if args.dataset is None:
        print("  (no --dataset; skipping the recorded part)")
    else:
        seconds, joints = read_episode(args.dataset, args.episode)
        print("  episode: {0}, {1} frames, {2:.3f} s at {3:.1f} fps".format(
            args.episode, len(seconds), seconds[-1], (len(seconds) - 1) / seconds[-1]))
        tool_position, tool_rotation = tool_path_from_joints(joints, chain)
        print("  tool   : p0 = ({0:+.4f}, {1:+.4f}, {2:+.4f}) m, closure "
              "|p_end - p_start| = {3:.4f} m".format(
                  tool_position[0][0], tool_position[0][1], tool_position[0][2],
                  float(np.linalg.norm(tool_position[-1] - tool_position[0]))))

        hand_position, hand_rotation = hand_from_tool(tool_position, tool_rotation, remap)
        recovered_position, recovered_rotation = tool_from_hand(
            hand_position, hand_rotation, tool_position[0], tool_rotation[0], remap)
        position_error = float(np.abs(recovered_position - tool_position).max())
        rotation_error = max(rotation_residual(recovered_rotation[index], tool_rotation[index])
                             for index in range(len(seconds)))
        print("  exact composition : max |dp| = {0:.3e} m, max |dR| = {1:.3e}".format(
            position_error, rotation_error))

        # The same loop with the prompt's simplified translation composition,
        # which is correct only while the hand's attitude is the identity.
        simple = tool_position[0] + (remap @ (tool_position - tool_position[0]).T).T
        simple_position, _ = tool_from_hand(
            simple, hand_rotation, tool_position[0], tool_rotation[0], remap)
        simple_error = float(np.abs(simple_position - tool_position).max())
        print("  p_hand0 + M*dt    : max |dp| = {0:.3e} m  (this is why the attitude "
              "term is carried)".format(simple_error))

        if position_error > 1e-9 or rotation_error > 1e-9:
            failures.append("the recorded-episode round trip is not exact")
        if simple_error <= 1e-6:
            print("  NOTE: this episode's attitudes are near identity, so the simple form "
                  "would also pass; the random path above is the discriminating check")

        # Registration against the mapper as written, frame pair by frame pair,
        # with the moving average window the node actually uses (10 samples).
        window = 10
        history = []
        averaged_position = [tool_position[0]]
        averaged_rotation = [tool_rotation[0]]
        lag = 0.0
        for index in range(1, len(seconds)):
            relative_translation, relative_rotation = mapper_increment(
                hand_position[index - 1], hand_rotation[index - 1],
                hand_position[index], hand_rotation[index], remap)
            history.append((relative_translation,
                            rotation_angle(relative_rotation) * _rotation_axis(relative_rotation)))
            if len(history) > window:
                history.pop(0)
            mean_translation = np.mean([item[0] for item in history], axis=0)
            mean_rotation = np.mean([item[1] for item in history], axis=0)
            averaged_position.append(averaged_position[-1] + mean_translation)
            averaged_rotation.append(
                _axis_angle_to_matrix(mean_rotation, float(np.linalg.norm(mean_rotation)))
                @ averaged_rotation[-1])
            lag = max(lag, float(np.linalg.norm(tool_position[index] - averaged_position[-1])))
        print("  window of {0}    : the target lags the recording by at most {1:.4f} m "
              "during transients; the sum of the increments is unchanged".format(window, lag))

    print("")
    if failures:
        print("FAILED: " + "; ".join(failures))
        return 1
    print("OK: the hand path this replay sends is the exact inverse of the mapper; every "
          "residual above is float round-off, well inside the 1e-9 this harness is allowed, "
          "and it was verified with no ROS and no socket")
    return 0


def _rotation_axis(matrix):
    """Unit axis of a rotation matrix (identity-safe)."""
    np = _np()
    m = np.asarray(matrix, dtype=float)
    axis = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.zeros(3)
    return axis / norm


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Replay a teleoperation episode (or a synthetic hand) into the "
                    "headset link.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--port", type=int, default=8766,
                        help="WebSocket port the headset link listens on; 8765 belongs to "
                             "the real headset and is never used by default")
    parser.add_argument("--dataset", default=None,
                        help="recorded dataset directory (lerobot layout)")
    parser.add_argument("--episode", type=int, default=0, help="episode index to replay")
    parser.add_argument("--synth", action="store_true",
                        help="a still hand instead of a dataset (the minimal case)")
    parser.add_argument("--rate", type=float, default=72.0, help="frames per second")
    parser.add_argument("--jitter", type=float, default=0.002,
                        help="uniform per frame jitter added to the interval, seconds")
    parser.add_argument("--noise-mm", type=float, default=0.0,
                        help="white noise added to the hand position, millimetres")
    parser.add_argument("--loop", action="store_true",
                        help="repeat the episode until --duration; the hand path continues, "
                             "so the tool target keeps accumulating")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds of stream to send (synth default 20, dataset default "
                             "one pass)")
    parser.add_argument("--passes", type=int, default=1,
                        help="episode repetitions when --loop is not given")
    parser.add_argument("--grip-at", type=float, default=1.0,
                        help="seconds into the stream at which the grip is held")
    parser.add_argument("--trigger-close-at", type=float, default=0.0,
                        help="press the trigger this many seconds before the end (0 = never)")
    parser.add_argument("--urdf", default=None, help="arm description to use for forward "
                                                     "kinematics; found automatically by default")
    parser.add_argument("--selftest", action="store_true",
                        help="verify the mapping numerically, with no ROS and no socket")
    parser.add_argument("--dry-run", action="store_true",
                        help="print what would be sent instead of opening a socket")
    parser.add_argument("--seed", type=int, default=None, help="seed for noise and jitter")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    bootstrap_interpreter(need_dataset=args.dataset is not None)

    if args.seed is not None:
        random.seed(args.seed)
        _np().random.seed(args.seed)

    if args.selftest:
        return selftest(args)

    if args.dataset is None and not args.synth:
        sys.stderr.write("nothing to replay: pass --dataset DIR (with --episode N), --synth, "
                         "or --selftest\n")
        return 2

    if args.synth and args.duration is None:
        args.duration = 20.0

    frames, description = build_frames(args)
    frames = schedule(frames, args)
    print("source   : {0}".format(description))
    print("stream   : {0} frames, {1:.2f} s at {2:.0f} Hz, jitter +/-{3:.1f} ms".format(
        len(frames), len(frames) / args.rate, args.rate, args.jitter * 1000.0))

    if args.dry_run:
        return dry_run(frames, args)
    if not frames:
        sys.stderr.write("no frames to send\n")
        return 2

    import asyncio
    started, intervals = asyncio.run(stream(frames, args))
    report_timing(frames, intervals, started, time.monotonic())
    return 0


if __name__ == "__main__":
    sys.exit(main())
