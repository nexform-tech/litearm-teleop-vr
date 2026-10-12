# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Tests for the headset frame parsing and the callback hand-off.

No socket, no headset and no rclpy: the module is loaded by path and frames are
handed to ``handle_message`` directly, which is why that method is public and
separate from the WebSocket handler. ``websockets`` is imported lazily by the
module, so it does not need to be present for these tests to run.

What is worth testing here is the tolerance, not the happy path. The headset is a
browser page that a person can reload, and a frame that arrives half-formed must
degrade to a defined value rather than raise inside the WebSocket handler, where
the only recovery available is dropping the client - which an operator
experiences as "the arm stopped following and nobody said why".
"""

import importlib.util
import json
import os
import sys

import pytest

from ament_index_python.packages import get_package_prefix


@pytest.fixture(scope="module")
def headset_link():
    """The module, loaded by path.

    It is installed as a program rather than as an importable package, because
    rosidl already installs a module named `litearm_vr_teleop` for the generated
    messages and two modules of the same name collide.
    """
    path = os.path.join(
        get_package_prefix("litearm_vr_teleop"), "lib", "litearm_vr_teleop", "headset_link.py")
    assert os.path.isfile(path), "headset_link.py is not installed at {0}".format(path)

    spec = importlib.util.spec_from_file_location("headset_link_under_test", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Recorder:
    """Collects the callbacks, so a test can assert on what the link published."""

    def __init__(self):
        self.frames = []

    def __call__(self, side, position, quaternion, rotation, grip, trigger, tracked, timestamp):
        self.frames.append(
            {
                "side": side,
                "position": position,
                "quaternion": quaternion,
                "rotation": rotation,
                "grip": grip,
                "trigger": trigger,
                "tracked": tracked,
                "timestamp": timestamp,
            })


def _frame(**overrides):
    controller = {
        "position": {"x": 0.1, "y": 1.2, "z": -0.3},
        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "rotation": {"x": 1.0, "y": 2.0, "z": 3.0},
        "gripActive": True,
        "trigger": 0.75,
        "tracked": True,
    }
    controller.update(overrides)
    return json.dumps({"rightController": controller, "timestamp": 1787641468151})


def test_publishes_the_right_hand(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    assert link.handle_message(_frame()) == 1
    assert len(recorder.frames) == 1

    published = recorder.frames[0]
    assert published["side"] == "right"
    assert published["position"] == pytest.approx((0.1, 1.2, -0.3))
    assert published["quaternion"] == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert published["grip"] is True
    assert published["trigger"] == pytest.approx(0.75)
    assert published["tracked"] is True
    assert published["timestamp"] == pytest.approx(1787641468151.0)
    assert link.frames_received == 1
    assert link.frames_rejected == 0


def test_publishes_one_frame_per_hand(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    message = json.dumps(
        {
            "leftController": {"position": {"x": 0, "y": 0, "z": 0}, "tracked": True},
            "rightController": {"position": {"x": 1, "y": 0, "z": 0}, "tracked": True},
        })
    assert link.handle_message(message) == 2
    assert sorted(frame["side"] for frame in recorder.frames) == ["left", "right"]
    # Counted once per message, not once per hand: the message is the frame.
    assert link.frames_received == 1


def test_rejects_a_message_that_is_not_json(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    assert link.handle_message("this is not json") == 0
    assert recorder.frames == []
    assert link.frames_rejected == 1
    assert link.frames_received == 0


def test_rejects_valid_json_without_a_controller(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    # Valid JSON, but not a controller frame. Counted as a rejection rather than
    # silently accepted, so a page wired up wrongly is visible in the counters.
    assert link.handle_message(json.dumps({"hello": "world"})) == 0
    assert link.frames_rejected == 1


def test_missing_fields_degrade_instead_of_raising(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    # An empty controller object: every field missing. The pose has to come out
    # defined, and `tracked` defaults to true because the field's absence means
    # the client does not report tracking at all, not that the hand is lost.
    assert link.handle_message(json.dumps({"rightController": {}})) == 1
    published = recorder.frames[0]
    assert published["position"] == (0.0, 0.0, 0.0)
    assert published["quaternion"] == (0.0, 0.0, 0.0, 1.0)
    assert published["grip"] is False
    assert published["trigger"] == 0.0
    assert published["tracked"] is True


def test_unparseable_numbers_degrade_to_zero(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    assert link.handle_message(_frame(trigger="not a number")) == 1
    assert recorder.frames[0]["trigger"] == 0.0


def test_untracked_hand_is_still_published(headset_link):
    recorder = Recorder()
    link = headset_link.HeadsetLink(recorder)

    # A lost hand is NOT the same as a still hand. Dropping the frame would tell
    # the consumer neither, so the frame is published and flagged instead.
    assert link.handle_message(_frame(tracked=False)) == 1
    assert recorder.frames[0]["tracked"] is False


def test_parse_controller_accepts_a_non_dict(headset_link):
    position, quaternion, rotation, grip, trigger, tracked = headset_link.parse_controller(None)

    assert position == (0.0, 0.0, 0.0)
    assert quaternion == (0.0, 0.0, 0.0, 1.0)
    assert rotation == (0.0, 0.0, 0.0)
    assert grip is False
    assert trigger == 0.0
    assert tracked is True
