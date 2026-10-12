# litearm_vr_teleop

VR teleoperation for the 7-axis LiteArm with a LiteGrip gripper, on ros2_control.

> Chinese documentation: [README.zh.md](README.zh.md). A five minute path through the
> stack: [QUICKSTART.zh.md](../docs/QUICKSTART.zh.md). Code comments and this file are the
> reference; the Chinese documents follow them.

A Quest 3 controller drives the arm and the gripper over the same hardware path
the MoveIt stack uses, with one deliberate difference: this stack runs a
**forward controller** instead of a trajectory controller, and does its planning
in the teleoperation node rather than in MoveIt. The headset's WebSocket link is
the part of `vr_control` that was kept; the planning arithmetic is ported from
its pybullet simulation into C++.

## Data flow

```text
Quest 3 WebXR page (~72 Hz), one JSON message per frame, plain ws
        |
        v  ws://<machine>:8765
headset_link.py            transport only: parse, publish, count
        |
        v  /vr_controller_frame  (litearm_vr_teleop/msg/VrControllerFrame)
vr_teleop_node             semantics, planning, command stream (C++, 100 Hz loop)
        |        |
        |        +--> /gripper_controller/gripper_cmd   (GripperCommand action)
        v
        /vr_forward_controller/commands  (Float64MultiArray, 7 positions)
        |
        v
VrForwardController        ros2_control controller: writes position AND velocity
        |
        v
LitearmSystem / mock       <ros2_control> hardware component
        |
        v
firmware daemon -> USB CDC -> arm          (or the mock, or a fake firmware)
```

## Why a forward controller, and why it writes velocity

The stock `forward_command_controller/ForwardCommandController` writes one
interface per joint, and on this arm that produces **an arm that accepts commands
and does not move**. The firmware reads the velocity field of its command frame
as the slew ceiling of the position reference; a controller that claims position
alone leaves the velocity interface at the value the hardware initialised it to,
which is zero, which freezes the reference. Nothing reports an error.

`VrForwardController` therefore derives from
`forward_command_controller::ForwardControllersBase` - it is a forward controller,
setpoint stream in, hardware interfaces out, no trajectory and no goal - and
overrides `update()` to write both interfaces, deriving each joint's velocity
from the setpoint stream itself and bounding it by `max_velocity`.

Two further things are overridden, both documented at the point of the override:

- `command_interface_configuration()`, because on the ros2_control installed here
  (2.54.2) the base class returns bare interface names (`["position",
  "velocity"]`) instead of `joint/interface` pairs, and the controller manager
  does not treat that as an empty configuration: it aborts the entire manager
  process.
- `on_configure()`, so the command subscription can record each message's arrival
  time. Staleness cannot be judged from the value, because a hold is a legitimate
  value. A stale command makes the controller hold at the measured pose with zero
  velocity, which on this firmware is the only true hold.

## Where the teleoperation behaviour lives

| Piece | File | What it decides |
|---|---|---|
| Increment mapping | `include/litearm_vr_teleop/teleop_math.hpp` | headset pose deltas to an end effector target: frame correspondence, moving average, per axis signs |
| Joint filter | same | low pass on the requested rate, then a per cycle step clamp |
| Point to point profile | `include/litearm_vr_teleop/joint_move.hpp` | the move to the start pose |
| Planning | `include/litearm_vr_teleop/teleop_planner.hpp` | forward kinematics, the Jacobian, and the two inverse kinematics backends |
| State machine | `src/vr_teleop_node.cpp` | grip and trigger edges, the headset watchdog, the stop service, what gets published when |
| Transport | `python/headset_link.py` | the WebSocket, frame parsing, and nothing else |

The split is drawn so that everything that can hurt somebody is C++ with tests,
and everything that cannot is Python.

### How the controller's rotation reaches the tool

Two frames are available, chosen by `rotation_mode` in `config/vr_teleop.yaml`,
and the difference is what an operator sees when they turn their hand:

- **`world` (default)** — the tool turns about the same world axis the controller
  turned about, by the same angle. If the operator rolls the controller, the tool
  rolls with it; if they swing its nose, the tool's nose swings the same way in
  the room. This is independent of the arm's pose and of how the controller is
  held.
- **`tool`** — the tool copies the controller axis for axis in its own frame. This
  is only the same motion while the tool's frame and the controller's frame stand
  in one particular relation to each other.

`world` is the default because that relation is a property of the arm's pose, not
of the rig. The reference's per axis signs were measured with the wrist rolled so
that the tool's own Y axis pointed straight down; at any other wrist roll the tool
frame is rolled with respect to the world by the difference, and the tool mode
answers a hand rotation with a motion rotated by exactly that much. At the
teleoperation start pose the wrist roll is about 95 degrees away from the pose
those signs were measured at, which is a rotation the operator feels immediately.
Both modes agree exactly at the measured pose, so `world` reproduces the measured
motion where it was measured and keeps it everywhere else.

The frame correspondence (the headset's reference frame is gravity aligned but
yawed with respect to the robot) stays a measured constant; it is per
installation, because it depends on which way the operator stands. The two
parameters above it are live, because the person who can tell whether a sign is
right is the one holding the controller:

```bash
ros2 param set /vr_teleop rotation_signs "[1.0, 1.0, -1.0]"
ros2 param set /vr_teleop rotation_mode tool
```

`rotation_signs` is `[1, 1, 1]` by default, which means "leave this mode as
measured": it is a correction applied on top of the mode's own baseline - nothing
for `world`, the reference's measured `[1, -1, -1]` for `tool`. That is what makes
switching modes live give the other *measured* behaviour rather than a tool frame
mode with its signs left out. A correction applies per axis in the frame the mode
reads in - robot axes for `world`, tool axes for `tool` - and exists for the cases
geometry cannot fix: a mirrored headset frame, or a controller held upside down
out of habit. A change takes effect on the next headset frame, and the arm holds
its pose until then.

### Jitter, and where the command stream's numbers come from

Three things decide whether the command stream is smooth, and they are separate
on purpose.

**The inverse kinematics is solved once per headset frame, not once per control
cycle.** The Cartesian target only changes when a frame arrives, but the solver
used to be re-run every cycle against it, seeded from the measured joints. A
solver that is not deterministic - and the SNS plugin is not, its search restarts
inside the 7 axis null space - then returned a slightly different answer each
time, a joint space wobble of a few hundredths of a radian per cycle. On this rig
that showed up exactly as the symptom it looks like: commanded positions that
move while the operator's hand does not. Between frames the last solution is
republished instead, which also matters for a second reason - the firmware wants
a continuous stream, not a new target - and the joint filter keeps easing toward
it, which turns one frame's step into a short ramp rather than a step. Measured
on the mock hardware with the hand held still: 0.00000 rad of commanded movement
per sample, against 0.049 rad in one cycle before, and with 0.3 mm of simulated
tracking noise the implied velocity request peaks at 0.14 rad/s against the
controller's 1.0 ceiling.

**The solver is seeded from the previous solution**, so the same target and the
same seed give the same answer and the redundant axis stays where the last
solution put it instead of wandering through the null space.

**The velocity the controller writes is the step over the interval between
command arrivals, not over the control cycle.** The firmware reads that field as
the slew ceiling of the position reference, so the honest value is the step
divided by the time the operator took to make it. This loop runs at 100 Hz and
the teleoperation node does not - it solves inverse kinematics per command - so
dividing by the cycle time multiplied every request by the ratio: at 23 Hz a
0.02 rad step became 2.0 rad/s instead of 0.46, which the 1.0 ceiling clipped, so
every step asked for full speed. That is the mechanism behind a velocity plot
that sits pinned at its limit.

`target_deadband_m` and `target_deadband_rad` (both 0 by default) keep target
changes below a threshold out of the solver entirely. Read the cost before
raising them: an increment below the deadband is dropped rather than accumulated,
because the target is re-based on the measured pose every frame, so a deadband of
0.5 mm at 72 Hz acts as a velocity threshold of 3.6 cm/s. Zero means the solver
sees every change.

### The velocity is a curve, not a step and a zero

Two things decide what the firmware sees in the velocity field, and both of them
had to be fixed before the field was worth plotting.

The controller runs at 100 Hz and the command stream does not: the node solves
inverse kinematics per command, so most control cycles have no arrival in them.
The velocity was recomputed every cycle, from the step of a setpoint that had not
changed since the last one - which is zero. The field therefore went to zero
between arrivals and back to the honest value on the next one. The firmware
advances the position reference at the ceiling it is given, so a ceiling of zero
half the time is an arm that moves half as far as it was told to, and a velocity
plot of a session shows a band of spikes. The velocity is now worked out on the
cycle a setpoint arrives and held until the next one.

The node then gives that stream an actual motion profile: `max_joint_acceleration`
(20 rad/s^2 by default) bounds how fast the speed may change and `max_joint_jerk`
(1000 rad/s^3) bounds how fast that change may itself change, so a move starts and
ends softly instead of asking for a step in speed. The limiter works out the
fastest speed a joint may hold and still stop on its target, from the stopping law
`v^2 / (2 a) + v a / (2 j) <= s` solved for the speed, and it includes the
target's own rate as a feed forward, so a hand moving steadily is followed at that
rate with no permanent offset. Tune `max_joint_acceleration` first: it is the
number the operator feels. A lower `max_joint_jerk` is not "smoother" - it also
delays the deceleration, which makes the arm overshoot instead of approaching.

### The tracking ratio, and the one switch that changes it

The mapping is velocity form and re-bases its target on the measured pose every
frame, which is what keeps a refused or clipped target from accumulating. The
cost is that motion the arm does not achieve in a cycle is dropped rather than
carried, and on a plant that lags - the real firmware, and the fake firmware in
the sim launch - the delivered displacement is a fraction of the hand's. On the
mock hardware (no lag) a 12 cm hand move delivers about 60 percent; the sim
launch's own documentation quotes about a quarter. An operator experiences that
as "I have to move my hand a long way before the arm does anything".

`reference_pose: commanded` accumulates on the last commanded pose instead, so
the arm's shortfall is carried and delivered as soon as the arm can take it, and
`max_target_lead_m` / `max_target_lead_rad` bound how far ahead the target may
run. It ships **off**: restoring the tracking ratio is a decision to make after
measuring on the hardware, not before, because it trades "the arm falls behind
and forgets" for "the arm catches up", and only the real arm can say which is
worse.

### The control loop does not run at its configured rate

`control_rate_hz` is 100, and on this machine the node achieves about **23 Hz**
while following, because each cycle does forward kinematics, an inverse
kinematics solve and the filter, and the SNS plugin does not fit in 10 ms. The
rate is measured from the command stream (the node publishes once per cycle), so
it can be checked with a `ros2 topic hz /vr_forward_controller/commands` while
the grip is held.

Everything scaled by `dt` is therefore scaled by the wrong number: the joint
filter's `alpha = dt/(dt + low_pass_tau)` becomes 0.68 instead of 0.33 (so it
passes two thirds of every step immediately), and the per cycle step limit
becomes 0.086 rad instead of 0.02. Two consequences worth knowing: the smoothing
is weaker than the constants suggest, and headset frames arrive faster than they
are consumed (72 against 23), so the averaging window covers about 0.4 s of wall
time instead of 0.14 s, which is part of the tracking shortfall above.

The honest fixes, in the order this was written: move the solve out of the
control loop (or run a cheaper backend), or lower `control_rate_hz` to what the
node can actually do and re-tune the `dt` scaled values against that number. What
is not a fix is leaving the configuration claiming 100 Hz while the coefficients
are computed from 43 ms.

## Two launches

```bash
ros2 launch litearm_vr_teleop teleop_virtual.launch.xml   # mock hardware
ros2 launch litearm_vr_teleop teleop_sim.launch.xml       # fake firmware, real protocol
```

Both start the same controllers and the same nodes. The difference is the last
centimetre, and it decides what the run can tell you.

**virtual** replaces the hardware with `mock_components/GenericSystem`: no
firmware, no daemon, no shared memory, no protocol, and the mock arm is at the
commanded pose immediately. Use it to watch the whole path run cheaply, and to
check *arithmetic*: axis directions, the return to the start pose, the gripper,
what a release does. Measured on this machine: the return to the start pose lands
within 0.0003 rad, and a 12 cm controller move produces a 12.5 cm end effector
move (the extra 4 percent is the designed smoothing tail).

**sim** runs the real firmware daemon in dry run mode: the real protocol, the real
framing, the real state reporting rate, and a fake firmware on a pty instead of
an arm. Nothing is attached to a serial port and `can0` is never opened. Use it to
check *behaviour that only exists with the real path*: that the firmware accepts
the command stream, that a continuous stream is produced at all (it has a 100 ms
watchdog), and that both command interfaces are written. It is the run that
distinguishes this controller from one that writes position only, because the
fake firmware freezes the reference on a zero velocity exactly as the real one
does.

Its own documentation warns against reading dynamics from it, and that warning is
real: the fake firmware is a first order kinematic model, so a following error or
a tracking ratio measured there means nothing. On this machine the sim run does
confirm that the arm moves and in the right direction, and shows the end effector
delivering about a quarter of the controller's motion - which is what a lagging
plant does to a velocity form mapping that re-bases its target on the measured
pose each frame, not a property of the command path.

Neither launch can reach real hardware. The dry run flag is hardcoded rather than
exposed as an argument, and the shared memory segment is named separately from
the arm stack's (`/litearm_vr_hw` rather than `/litearm_hw`) so this stack cannot
attach to a segment belonging to a running arm stack. A real hardware launch is
one copy of `teleop_sim.launch.xml` with `dry-run` removed from the daemon
arguments and the shared memory name changed back - and, before it is used, the
four items in "Before real hardware" below.

There is deliberately no `move_group`. Planning is inside `vr_teleop_node`, so a
planner node here would be a second, unused planning entry.

## Configuration

`config/vr_controllers.yaml` - the controller declarations and their parameters.
Note that a command interface is owned by exactly one active controller, so this
file replaces the arm stack's controller file rather than adding to it: the
trajectory controller claims the same interfaces and cannot run at the same time.

`config/vr_teleop.yaml` - parameters for the two nodes. Neither node decides how
fast the arm may move; that belongs to the controller and to the firmware, so a
mistake in this file cannot raise the hardware's speed limit.

The inverse kinematics solver, its four tuning values, and the teleoperation start
pose exist in two places: here, and in `litearm_vr_moveit_config` (which is where
MoveIt reads them from, and where the SRDF names the start pose). They are
repeated rather than imported because a MoveIt kinematics file has no node name at
its top level and cannot be handed to a node as a parameter file. The duplication
is guarded by `test/test_kinematics_consistency.py`, which fails if the two drift
apart. The same file checks that no XML comment anywhere in the package contains
two hyphens in a row, which is not pedantry: a dashed underline under a heading is
easy to write, makes the file stop being XML, and the resulting error points at a
line of prose rather than at the offending characters.

## Building and running

```bash
# one shell, once
source /opt/ros/humble/setup.bash
source <arm workspace>/install/setup.bash   # litearm_ros2_control, litegrip_ros2_control, ...

cd <colcon workspace>
colcon build --symlink-install
source install/setup.bash

ros2 launch litearm_vr_teleop teleop_virtual.launch.xml
```

The headset connects to `ws://<this machine>:8765`. The port can be moved with
`headset_port:=`. Note that `vr_control`'s `ws_echo_server.py` also defaults to
8765, so if it is running, this stack will refuse to start until one of them
moves: `headset_link` reports the bind failure and exits rather than staying alive
without a link.

Operation: press the grip to follow. The arm first returns to the teleoperation
start pose at equal joint speed (joints with less to travel arrive first, which is
the SDK's `movej` semantics rather than `movej_sync`), then follows the
controller. Release the grip to hold. Pull the trigger to close the gripper,
release it to open.

```bash
ros2 service call /vr_teleop/stop std_srvs/srv/Trigger
ros2 service call /vr_teleop/start std_srvs/srv/Trigger
```

`~/stop` freezes everything and requires `~/start`. Separately, if no headset
frame arrives for `headset_timeout_s` (0.5 s by default) the node holds and the
next grip press re-anchors: a headset that comes back at a different pose must not
be treated as continuous with the pose before it.

## A name the launch resolves for you

The virtual launch needs the arm description **without** any ros2_control block,
and the package providing it is `litearm_description`. xacro cannot compose a
`$(find ...)` from a variable, so `litearm_vr_moveit_config` detects the name at
build time through the ament index, records it, and the launch passes the resolved
path to xacro. The banner prints what was used:

```text
mock hardware: true
arm description: litearm_description
```

Detection is not by name alone: the candidate's URDF is read and must declare
`joint1` and `ee_link`, which is the generation this whole package is written
against. An install of the superseded CAD export of this arm (joints
`Joint1..Joint7`, links `Link1..Link7`, flange `ee_frame_link` — a different arm,
not a rename) stops the build with that reason, instead of producing an URDF that
urdfdom refuses. Override with `arm_description_package:=<name>`. See the moveit
config package's README.

## Which interpreter runs the Python node

`websockets` is not part of the system Python on the machine this was written for,
and the ROS installation does not provide it either. `headset_link.py` checks for
it at startup and re-executes itself into the interpreter named by
`LITEARM_VR_PYTHON` (default `$HOME/venvs/metavr-arm/bin/python`) if it is
missing. The re-execution is in place, so launch's process handling and SIGINT
delivery are unchanged, and it is skipped entirely when the module is already
available. The launch argument `headset_python` sets that environment variable.

## Tests

```bash
colcon test --packages-select litearm_vr_teleop
colcon test-result
```

The C++ tests cover the increment mapping, the joint filter and the point to point
profile against an idealised plant: that a still controller keeps the arm still,
that a constant controller velocity is tracked with the same velocity, that the
smoothing tail after a sudden stop is bounded, that a joint with less to travel
arrives first, and that the filter does not lose gain on the way. The Python tests
cover frame parsing tolerance and the callback hand off, which is where a
half-formed frame from a browser page would otherwise become a dropped client.

The tests do not prove the stack works; the two launches do. What the tests
prove is that a sign flip or a gain error is caught before it reaches a motor.

## Before real hardware

Four things, in the order they will bite:

1. **The gripper will not move.** Its controller refuses to send any motion frame
   while `litegrip_max_feedback_velocity_rad_s` is the fail-closed default. It has
   to be calibrated on the actual gripper (run it through real working conditions,
   take the peak feedback velocity, add margin). The launch files pass a derived
   value that is only valid for the simulated plant, with a comment saying so.
2. **The tracking ratio.** A velocity form mapping that re-bases its target on the
   measured end effector pose will under-deliver against any plant that lags. If
   an operator finds the arm slow, the change is to accumulate the target on the
   previously *commanded* pose instead, with a windup guard for the case where the
   arm cannot follow. That is a decision, not a bug fix, and it should be made
   after measuring the real arm rather than before.
3. **The speed ceiling.** `max_velocity` in the controller is 1.0 rad/s, well
   below the firmware's own limit. It is a guard rail, not a tuning knob.
4. **The start pose.** It is what the SRDF calls `grasp_ready`, and the arm will
   travel to it before following anything. Check it is clear of obstacles from the
   poses the arm actually starts in, not just from the demo's.

## What is not here

- A real hardware launch, on purpose (see above).
- `move_group`, and with it any MoveIt planning: the planner is the node.
- Any change to `litearm_ros2_control`, `litegrip_ros2_control`, or the arm's
  MoveIt configuration. This package depends on them and replaces none of them.
- Dual arm support. The mapping is configured for one upright arm; the frame
  message already carries a side per hand, so a second arm needs a second
  controller instance and the side configuration that `vr_control` measured, not a
  new message.
