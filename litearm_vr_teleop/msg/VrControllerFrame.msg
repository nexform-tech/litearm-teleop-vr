# One controller frame from the VR headset, forwarded verbatim by the Python
# transport node (litearm_vr_teleop/headset_link.py).
#
# The field set mirrors the JSON the Quest 3 WebXR page already sends, one
# message per hand per frame. Nothing is pre-digested: grip and trigger are
# carried as the raw button and analogue values, and all teleoperation
# semantics (edge detection, debouncing, the velocity-form increment mapping,
# inverse kinematics) happen on the C++ side. That split is deliberate. The
# Python node can then be replaced, restarted or replayed without changing what
# the robot does, and the behaviour that matters stays in the language and the
# tests where it can be verified.

std_msgs/Header header

# Which hand produced this frame: "left" or "right". The single arm setup uses
# "right" only, but the field exists so a second arm needs no new message.
string side

# Whether the headset is currently tracking the controller. A frame with
# tracked false carries stale values and must be ignored rather than
# interpreted as a zero motion.
bool tracked

# Controller pose in the WebXR reference space (local-floor):
#   +x right, +y up, +z back, so forward is -z.
# The values are metres and a unit quaternion, unconverted: the mapping into
# the robot frame is axis_remap / translation_remap, not something this message
# should have already applied.
geometry_msgs/Point position
geometry_msgs/Quaternion quaternion

# Euler angles as sent by the page, in degrees. Kept for diagnostics only; the
# teleoperation path uses position and quaternion.
geometry_msgs/Vector3 rotation

# Grip button. Rising edge starts following (after a point to point move to the
# teleop start pose), falling edge pauses and holds.
bool grip_active

# Trigger analogue in [0, 1]. Rising edge closes the gripper, falling edge opens
# it. The analogue value is not used continuously: the gripper is commanded
# through a goal based action where closed onto an object and stopped counts as
# success, and streaming goals at frame rate would fight that judgement.
float64 trigger

# Milliseconds, as sent by the page. Diagnostics only: the C++ side times its
# edges and its watchdog off a local steady clock, because the headset clock is
# not synchronised and jumps on reconnect.
float64 source_timestamp_ms
