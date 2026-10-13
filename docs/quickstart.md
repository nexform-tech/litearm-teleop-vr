# Quickstart

VR teleoperation for the LiteArm seven axis arm and its LiteGrip gripper. The same
link has two modes below: **simulated by default** (`mock_components` hardware,
nothing moves), and the **real hardware** path with `dry_run:=false`, which loads
the drivers. See section 6 for that one.

> Language convention: code comments and the English documents are English; this
> file and [`quickstart.zh-CN.md`](quickstart.zh-CN.md) are both kept, and where
> the two disagree the English one is the reference.

---

## 1. Thirty seconds

```bash
# (1) the arm's own workspace first: this package depends on litearm_ros2_control,
#     litegrip_ros2_control and the arm description, which live there
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash

# (2) build (about 30 seconds the first time)
cd ~/teleop_ws            # the colcon workspace this repository is cloned into
colcon build --symlink-install
source install/setup.bash

# (3) start the simulated stack, without RViz, to check the link first
ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py use_rviz:=false   # dry_run:=true by default
```

These three lines mean it came up:

```text
[vr_teleop]: VR teleop ready: ... planning backend 'dls'
[headset_link]: headset link serving on ws://0.0.0.0:8765
[spawner]: Configured and activated joint_state_broadcaster / vr_forward_controller / gripper_controller
```

## 2. Confirm the three controllers are really alive

If `ros2 control` is not an available subcommand, ask the service instead:

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1   # the domain this stack locks itself to
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers "{}" | tr ',' '\n' | grep -E "name=|state="
```

Expected: `joint_state_broadcaster`, `vr_forward_controller` and `gripper_controller`,
all three in `state='active'`.

Then check that the command topic has exactly **one** subscriber - two subscribers
means the commands are being written twice:

```bash
ros2 topic info -v /vr_forward_controller/commands | grep -E "Type:|Subscription count"
# expected: Type: std_msgs/msg/Float64MultiArray · Subscription count: 1
```

## 3. Connect the headset

The headset connects to `ws://127.0.0.1:8765` (use the address of the machine
running this stack when the headset is elsewhere). Move the port with
`headset_port:=8876`.

> **The reference implementation's `ws_echo_server.py` also defaults to 8765.**
> While both run, this stack's `headset_link` **says why and exits** rather than
> pretending to serve:
> `headset link could not serve on 0.0.0.0:8765: [Errno 98] address already in use`.
> Either stop the other one or give this stack another port.
>
> To see who holds 8765: `ss -tlnp | grep 8765`

### Controls

| Action | Result |
|---|---|
| Press the grip | The arm first returns to the teleoperation start pose (`grasp_ready`) at equal joint speed, then follows. Equal speed means the joints with less to travel arrive first: the SDK's `movej` semantics, not `movej_sync` |
| Release the grip | Holds where it is (the measured position and a velocity of zero, which is this firmware's only real way to stand still) |
| Pull the trigger | The gripper closes |
| Release the trigger | The gripper opens (a press shorter than 0.5 s is ignored, so "let go before it gripped" does not happen) |

External hold and resume:

```bash
ros2 service call /vr_teleop/stop  std_srvs/srv/Trigger   # freeze; only ~/start resumes
ros2 service call /vr_teleop/start std_srvs/srv/Trigger
```

There is also a headset watchdog: after 0.5 s without a frame (`headset_timeout_s`)
the arm holds, and the next grip press **re-anchors**, because the pose after a
reconnect is not continuous with the pose before it.

## 4. Testing without a headset

Feed synthetic frames into the WebSocket. This path is verified end to end: it
drives the return to the start pose, following, and the gripper. It needs the
`websockets` module, so run it with an interpreter that has it:

```bash
cat > /tmp/vr_fake_hand.py <<'EOF'
import asyncio, json, time, sys
import websockets

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765

def frame(x, y, z, grip, trigger):
    return json.dumps({"rightController": {
        "position": {"x": x, "y": y, "z": z},
        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "gripActive": grip, "trigger": trigger, "tracked": True},
        "timestamp": int(time.time() * 1000)})

async def main():
    async with websockets.connect(f"ws://127.0.0.1:{PORT}") as ws:
        async def hold(seconds, pose, grip, trigger):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                await ws.send(frame(pose[0], pose[1], pose[2], grip, trigger))
                await asyncio.sleep(1 / 30)

        origin = (0.0, 1.0, 0.0)
        await hold(10.0, origin, True, 0.0)            # grip: the arm returns to the start pose
        for i in range(61):                            # push 12 cm to the operator's right
            await ws.send(frame(i / 60.0 * 0.12, 1.0, 0.0, True, 0.0))
            await asyncio.sleep(1 / 30)
        await hold(3.0, (0.12, 1.0, 0.0), True, 0.0)   # stop, and let it settle
        await hold(2.0, (0.12, 1.0, 0.0), True, 0.9)   # trigger: the gripper closes
        await hold(3.0, (0.12, 1.0, 0.0), True, 0.0)   # release: it opens
        await hold(2.0, (0.12, 1.0, 0.0), False, 0.0)  # release the grip: hold

asyncio.run(main())
EOF

# an interpreter that has websockets (the system python3 usually does not;
# headset_link.py switches to whatever LITEARM_VR_PYTHON points at)
# the argument is the headset_port you launched with (8765 by default)
$HOME/venvs/vr/bin/python /tmp/vr_fake_hand.py 8765
```

Watch the joint angles while it runs - **by name**, for the reason below:

```bash
ros2 topic echo --once /joint_states 2>/dev/null | python3 -c "
import sys, re
t = sys.stdin.read()
names = re.search(r'name:\n((?:- .*\n)+)', t)
vals  = re.search(r'position:\n((?:- .*\n)+)', t)
n = [x[2:].strip() for x in names.group(1).strip().splitlines()] if names else []
v = [float(x[2:]) for x in vals.group(1).strip().splitlines()] if vals else []
d = dict(zip(n, v))
for j in ['joint1','joint2','joint3','joint4','joint5','joint6','joint7','gripper_opening_joint']:
    print('{0:24s} {1:+.5f}'.format(j, d.get(j, float('nan'))))"
```

> **The order of the arrays in `/joint_states` is not `joint1..joint7`, and it can
> differ between runs.** Two launches of the same stack were measured to produce
> the same set in different orders:
> `gripper_opening_joint, joint2, joint3, joint5, joint6, joint1, joint4, joint7`
> and `joint2, joint3, joint5, joint6, joint1, joint4, joint7, gripper_opening_joint`.
> The order comes from the order the resource manager enumerates interfaces; the
> broadcaster's `joints` parameter only filters which joints are published, not
> what order they come in. **Any consumer that indexes `position[i]` (a dataset
> storing `observation.state[8]`, for instance) gets a set of joint angles that
> looks like data and is not**, and it will differ between runs. Always read by
> name.

What to look for (the simulated arm starts from all zeros, so watch the values
after the grip press):

- about ten seconds after the grip press, `joint1..7` should be
  `[0, -0.6458, 0, -1.9897, 0, -0.8378, 0]` (the start pose);
- during the 12 cm push, `joint1` turns positive and `joint3`/`joint5` negative
  (the controller moves right, which is base `+x`);
- the trigger sets `gripper_opening_joint` to `0.0`, and releasing it returns
  `0.07479`.

## 5. Seeing it

```bash
ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py   # RViz by default
```

RViz shows `RobotModel`, `TF`, `Grid` and `Axes`, fixed to `base_link`.
There is deliberately **no** MoveIt MotionPlanning panel: this stack plans inside
`vr_teleop_node` and never starts `move_group`.

## 6. Real hardware

```bash
ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py dry_run:=false
```

`dry_run` is the **only** mode switch (default true, which simulates and touches
nothing). Set false and the launch loads the hardware drivers: the arm's direct USB
component enables its motors at configure time and the gripper component opens its
motor gate. A grip press then returns the arm to its start pose at 0.5 rad/s.
Before running it, read "Before real hardware" in the package README, keep the
emergency stop in reach, and support the arm.

The three launch files differ only in what they control: `teleop_arm`,
`teleop_gripper`, `teleop_arm_gripper`. Whatever is not controlled stays
simulated - a real component with nobody commanding it is a motor being held
energised.

## 7. When it does not move, read the symptom

| Symptom | Look at first |
|---|---|
| Not all three controllers came up | The `spawner` lines in the launch log; the ERROR line right after `Failed loading controller` is the real reason |
| `vr_forward_controller` reports `parameter 'joints' is empty` | The controller node did not get its own parameters. Check that the `spawner` has `--param-file`, and that `ros2_control_node` was not given an explicit `name=` (which injects a `__node:=` remap that renames the controller node too, so it reads the manager's parameters) |
| The manager exits, log says `was not find in the interface name` | The command interface name is not `joint/interface`. ros2_control 2.54.2's forward base class returns bare names, so `command_interface_configuration()` has to be overridden |
| With `dry_run:=false`, `LitearmSystem` fails to open the shared memory | The controller started before the daemon. This stack serialises them with a two second delay (`<timer period="2.0">`); the segment is named `/litearm_vr_hw` |
| The arm receives commands and does not move | Every velocity in the command frame is zero. Check that something publishes on the topic, and that the controller's `max_velocity` is not zero |
| The gripper does not move | Its driver is fail-closed: without `litegrip_max_feedback_velocity_rad_s` it refuses to send any motion frame. The launch passes a derived value when simulating; on real hardware it has to be calibrated by measurement |
| The headset is connected and the arm does not move | `headset_link` prints "no headset is connected / last frame N s ago" every 10 s; then check that the grip really was pressed (a rising edge, and `tracked` must not be false) |
| Indexing `/joint_states` gives angles that look like data but are wrong | The order is not `joint1..7` and is not fixed. Read by name (section 4); this one never raises an error and does not stay the same between runs |
| RViz is up but there is no robot | `RobotModel` comes from `/robot_description` (transient local, latched); check that `robot_state_publisher` is running |
| Build fails with `package 'litearm' not found` from xacro | No description package in the environment. `source ~/arm_ws/install/setup.bash` first. **Copying `share/` into the install tree by hand is not an install**; it never reaches the ament index |
| Build fails with `needs an arm description that declares joint1..joint7 and ee_link` | The installed description is another generation of this arm (`Link1..Link7`, `ee_frame_link`). The error lists the candidates it found and why each was refused; the one needed declares `joint1..7` and `ee_link` |
| The banner's `arm description:` is not the package you expected | Name it: `arm_description_package:=<package>` |
| `ros2 topic list` is empty in another terminal | This stack locks `ROS_DOMAIN_ID=42` and `ROS_LOCALHOST_ONLY=1`; export both in that terminal too (the launch banner prints them) |

## 8. Shutting down

Ctrl-C on the launch is enough; it runs in its own process group. Check for
leftovers:

```bash
pgrep -af 'vr_teleop_nod[e]|ros2_control_nod[e]|headset_lin[k]|rviz[2]|litearm_hw_daemo[n]'
ls /dev/shm/ | grep litearm          # empty if only the simulated stack ran
```

None of the three launches uses a shared memory segment (the arm's direct USB
component opens its own serial port). A `litearm*` segment left in `/dev/shm` came
from another stack; delete it.
