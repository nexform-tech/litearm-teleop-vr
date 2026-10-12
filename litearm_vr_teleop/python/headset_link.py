#!/usr/bin/env python3
# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""The link to the VR headset: one class, and the node that publishes it.

This is the part of vr_control's ``metavr/scripts/ws_echo_server.py`` that talks
to the headset, with the parts that made claims about a robot left behind. What is
kept is the wire format, which the Quest 3 WebXR page already speaks, and the
handling of the ways that link misbehaves: a frame that is not JSON, a hand that
is not tracked, a client that disconnects mid-session.

Class ``HeadsetLink``
---------------------
Owns the WebSocket server and turns each frame into one callback per hand. It
runs its own event loop on a daemon thread, so the caller's thread stays free for
the ROS executor. That direction is chosen rather than the reverse because the ROS
side owns the process lifecycle: parameters, services and shutdown all behave
normally when the event loop is the thing in the background.

What is deliberately NOT in this file
-------------------------------------
No teleoperation semantics and no robot knowledge. Grip and trigger edges,
debouncing, the increment mapping, inverse kinematics and the gripper all belong
to ``vr_teleop_node``. The reason is not tidiness: the behaviour that can hurt
somebody has to be testable, and it is testable on the C++ side without a
headset, a WebSocket, or a Python interpreter in the loop.

Wire format (one JSON text message per frame)::

    {
      "leftController":  {"position": {"x","y","z"},
                          "quaternion": {"x","y","z","w"},
                          "rotation": {"x","y","z"},
                          "gripActive": false, "trigger": 0.0, "tracked": true},
      "rightController": {... same shape ...},
      "timestamp": 1787641468151
    }

``position`` is metres in the WebXR reference space (x right, y up, z back, so
forward is -z), ``rotation`` is Euler angles in degrees and unused, ``trigger`` is
analogue in [0, 1], and ``timestamp`` is milliseconds on the headset's clock.
None of it is converted here: converting is the C++ side's job, and doing it in
two places is how the two drift apart.

Which interpreter runs this
--------------------------
``websockets`` is not part of the system Python on this machine and the ROS
installation does not provide it either. The interpreter that has it is the
project's virtual environment, so this script re-executes itself into it at
startup, keeping its process id, which leaves launch's process handling and its
SIGINT delivery unchanged. The interpreter is chosen by ``LITEARM_VR_PYTHON``,
defaulting to ``$HOME/venvs/metavr-arm/bin/python``, and the bootstrap is skipped
entirely if the current interpreter already has ``websockets`` - so an
installation that does have it system wide pays nothing.

``websockets`` is imported lazily, inside the methods that need it, so that this
module can be imported and tested by an interpreter that does not have it. That
is not a trick to avoid a dependency: the frame parsing and the callback hand-off
are worth testing, and they need no socket.
"""

import asyncio
import json
import os
import sys
import threading
import time

# Guard against re-executing forever if the chosen interpreter also lacks the
# module. The value is opaque on purpose: it only has to exist.
_BOOTSTRAP_ENV = "LITEARM_VR_LINK_BOOTSTRAPPED"
_DEFAULT_PYTHON = os.path.join(
    os.path.expanduser("~"), "venvs", "metavr-arm", "bin", "python")

# Controller field name in the JSON -> the `side` published on the frame.
HAND_TO_SIDE = {"leftController": "left", "rightController": "right"}

# A frame describing a hand the headset is not tracking still needs a pose. Zero
# is used, and `tracked` false is what tells the consumer not to believe it.
_UNTRACKED_POSE = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0},
}


def bootstrap_interpreter():
    """Re-exec into an interpreter that has `websockets`, if this one does not."""
    if os.environ.get(_BOOTSTRAP_ENV) == "1":
        return
    try:
        import websockets  # noqa: F401
        return
    except ImportError:
        pass

    candidate = os.environ.get("LITEARM_VR_PYTHON") or _DEFAULT_PYTHON
    if not os.path.isfile(candidate):
        sys.stderr.write(
            "headset_link: the interpreter {0} does not provide 'websockets', and the "
            "fallback interpreter {1} does not exist.\n"
            "Set LITEARM_VR_PYTHON to an interpreter that has it, or install "
            "python3-websockets.\n".format(sys.executable, candidate))
        raise SystemExit(2)

    environment = dict(os.environ, **{_BOOTSTRAP_ENV: "1"})
    sys.stderr.write(
        "headset_link: re-executing into {0} (websockets is not available in {1})\n".format(
            candidate, sys.executable))
    os.execve(candidate, [candidate, os.path.abspath(__file__)] + sys.argv[1:], environment)


def _as_float(value, fallback=0.0):
    """Read a number out of a frame without letting a bad frame kill the link."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def parse_controller(controller):
    """One controller object -> (position, quaternion, rotation, grip, trigger, tracked).

    Tolerant on purpose. The headset is a browser page that a person can reload
    mid-session, and a frame with a missing field has to degrade to a defined
    value rather than raise inside the WebSocket handler, where the only recovery
    available is dropping the client.
    """
    source = controller if isinstance(controller, dict) else {}
    position = source.get("position") or _UNTRACKED_POSE["position"]
    quaternion = source.get("quaternion") or _UNTRACKED_POSE["quaternion"]
    rotation = source.get("rotation") or _UNTRACKED_POSE["rotation"]
    return (
        (_as_float(position.get("x")), _as_float(position.get("y")), _as_float(position.get("z"))),
        (
            _as_float(quaternion.get("x")),
            _as_float(quaternion.get("y")),
            _as_float(quaternion.get("z")),
            _as_float(quaternion.get("w"), 1.0),
        ),
        (_as_float(rotation.get("x")), _as_float(rotation.get("y")), _as_float(rotation.get("z"))),
        bool(source.get("gripActive", False)),
        _as_float(source.get("trigger", 0.0)),
        bool(source.get("tracked", True)),
    )


class HeadsetLink:
    """A WebSocket server that turns headset frames into callbacks.

    ``on_frame(side, position, quaternion, rotation, grip_active, trigger,
    tracked, source_timestamp_ms)`` is called once per hand per frame, from the
    event loop thread. It must be cheap and must not block: it is on the critical
    path of a 72 Hz stream.

    Safe to ``start()`` before a headset has ever connected, and ``stop()`` is
    idempotent, because a launch file stops nodes it may not have finished
    starting.

    The link publishes enough state to tell the two silent failures apart: no
    client has connected, versus a client connected and no frames arriving. Both
    look like an arm that does not move.
    """

    def __init__(self, on_frame, host="0.0.0.0", port=8765, on_client=None, logger=None):
        self._on_frame = on_frame
        self._on_client = on_client
        self._host = host
        self._port = int(port)
        self._log = logger

        self._loop = None
        self._server = None
        self._thread = None
        self._started = threading.Event()
        self._start_error = None

        self.frames_received = 0
        self.frames_rejected = 0
        self.clients_connected = 0
        self.last_frame_time = None

    # ------------------------------------------------------------------ status

    @property
    def port(self):
        return self._port

    @property
    def start_error(self):
        """Set when the server could not bind, e.g. because the port was taken."""
        return self._start_error

    def seconds_since_last_frame(self):
        """Seconds since the last accepted frame, or None if there has been none."""
        if self.last_frame_time is None:
            return None
        return time.monotonic() - self.last_frame_time

    # -------------------------------------------------------------- life cycle

    def start(self, timeout=5.0):
        """Start serving in the background. Returns True once the port is bound.

        Binding happens before this returns, so a launch file that starts this
        node learns immediately whether the port was available, rather than the
        failure surfacing later as an unexplained silence.
        """
        if self._thread is not None:
            return True

        self._thread = threading.Thread(target=self._run, name="headset-link", daemon=True)
        self._thread.start()
        self._started.wait(timeout)
        return self._start_error is None and self._server is not None

    def stop(self):
        """Stop serving. Safe to call more than once and before ``start()``."""
        loop = self._loop
        if loop is not None and not loop.is_closed():
            # Stopping the loop from another thread is only legal through
            # call_soon_threadsafe; doing it directly raises intermittently.
            loop.call_soon_threadsafe(loop.stop)
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None
        self._server = None

    # ----------------------------------------------------------------- internals

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
            self._loop.run_forever()
        except OSError as error:
            # Almost always "address already in use". Recorded rather than raised:
            # this is a background thread, where raising only produces a traceback
            # nobody reads.
            self._start_error = str(error)
            self._error("headset link could not serve on %s:%d: %s", self._host, self._port, error)
            self._started.set()
        finally:
            try:
                self._loop.close()
            except Exception:  # pragma: no cover - shutdown is best effort
                pass

    async def _serve(self):
        import websockets

        self._server = await websockets.serve(self._handler, self._host, self._port)
        self._info("headset link serving on ws://%s:%d", self._host, self._port)
        self._started.set()

    async def _handler(self, websocket):
        from websockets.exceptions import ConnectionClosed

        self.clients_connected += 1
        self._info("headset connected")
        if self._on_client is not None:
            self._on_client(True)
        try:
            async for message in websocket:
                self.handle_message(message)
        except ConnectionClosed:
            pass
        finally:
            self.clients_connected -= 1
            self._info("headset disconnected")
            if self._on_client is not None:
                self._on_client(False)

    def handle_message(self, message):
        """Parse one text frame and call back once per hand.

        Public, and separated from the socket, because this is the part worth
        testing and it has nothing to do with sockets. Returns the number of
        hands published.
        """
        try:
            data = json.loads(message)
        except (ValueError, TypeError):
            self.frames_rejected += 1
            # A malformed frame is a client bug, not a reason to drop the client:
            # the next frame is very likely fine.
            return 0
        if not isinstance(data, dict):
            self.frames_rejected += 1
            return 0

        timestamp = _as_float(data.get("timestamp", 0.0))
        published = 0
        for hand, side in HAND_TO_SIDE.items():
            if hand not in data:
                continue
            position, quaternion, rotation, grip, trigger, tracked = parse_controller(data[hand])
            self._on_frame(
                side, position, quaternion, rotation, grip, trigger, tracked, timestamp)
            published += 1

        if published == 0:
            # Neither hand present, which is a different cause from a malformed
            # frame: the page is sending valid JSON that is not a controller frame.
            self.frames_rejected += 1
            return 0

        self.frames_received += 1
        self.last_frame_time = time.monotonic()
        return published

    # ------------------------------------------------------------------ logging

    def _info(self, message, *args):
        if self._log is not None:
            self._log.info(message % args if args else message)

    def _error(self, message, *args):
        if self._log is not None:
            self._log.error(message % args if args else message)


# ------------------------------------------------------------------------------
# The ROS node. Everything below needs rclpy, which is imported after the
# bootstrap has had a chance to re-execute into an interpreter that has it.
# ------------------------------------------------------------------------------


class RosLogger:
    """Adapts the rclpy logger to the plain ``info`` / ``error`` the link uses."""

    def __init__(self, node):
        self._node = node

    def info(self, message):
        self._node.get_logger().info(message)

    def error(self, message):
        self._node.get_logger().error(message)


def _parameter(node, name, fallback):
    """Read a parameter, declaring it only if the launch file did not supply it.

    A parameter supplied as an override already exists on the node, and its type
    comes from the launch file, which cannot always produce the type this node's
    default would have. Declaring it again would either raise or silently change
    the type underneath.
    """
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return node.declare_parameter(name, fallback).value


def main(args=None):
    import rclpy
    from rclpy.node import Node

    from geometry_msgs.msg import Point, Quaternion, Vector3
    from std_msgs.msg import Header

    from litearm_vr_teleop.msg import VrControllerFrame

    class HeadsetLinkNode(Node):
        """Publishes one frame per hand, and reports whether anything is arriving."""

        def __init__(self):
            super().__init__("headset_link")

            host = str(_parameter(self, "host", "0.0.0.0"))
            port = int(_parameter(self, "port", 8765))
            topic = str(_parameter(self, "frame_topic", "/vr_controller_frame"))
            report_period = float(_parameter(self, "report_period_s", 10.0))

            self._publisher = self.create_publisher(VrControllerFrame, topic, 10)
            self._link = HeadsetLink(
                on_frame=self._on_frame, host=host, port=port, logger=RosLogger(self))

            if not self._link.start():
                # Binding failed. Exiting is right: this node's whole job is to
                # bring up a link, and one that stayed alive without it would look
                # healthy while the headset could never connect.
                # Pre-formatted on purpose: an rclpy logger takes one message and
                # no format arguments, so passing them through is a TypeError at
                # the exact moment the operator most needs to read the reason.
                self.get_logger().error(
                    "the headset link could not serve on {0}:{1} ({2})".format(
                        host, port, self._link.start_error or "unknown reason"))
                raise RuntimeError("headset link failed to start")

            self.get_logger().info(
                "connect the headset to ws://<this machine>:{0}; publishing {1}".format(
                    port, topic))
            if report_period > 0.0:
                self.create_timer(report_period, self._report)

        def _on_frame(self, side, position, quaternion, rotation, grip, trigger,
                      tracked, timestamp):
            """Called from the link's event loop thread: must stay cheap."""
            message = VrControllerFrame()
            message.header = Header()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "vr_headset"
            message.side = side
            message.tracked = tracked
            message.position = Point(x=position[0], y=position[1], z=position[2])
            message.quaternion = Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            message.rotation = Vector3(x=rotation[0], y=rotation[1], z=rotation[2])
            message.grip_active = grip
            message.trigger = trigger
            message.source_timestamp_ms = timestamp
            self._publisher.publish(message)

        def _report(self):
            """Say whether frames are arriving.

            Without this, the only symptom of a headset that is connected but not
            sending (a page left on the wrong mode, a tab that lost the XR
            session) is an arm that does not move, which is indistinguishable
            from several other problems.
            """
            idle = self._link.seconds_since_last_frame()
            if self._link.clients_connected == 0:
                self.get_logger().warn(
                    "no headset is connected (received {0} frames so far)".format(
                        self._link.frames_received))
            elif idle is None:
                self.get_logger().warn("a headset is connected but no frame has arrived yet")
            elif idle > 2.0:
                self.get_logger().warn(
                    "a headset is connected but the last frame was {0:.1f} s ago".format(idle))

        def destroy_node(self):
            self._link.stop()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = HeadsetLinkNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    bootstrap_interpreter()
    main()
