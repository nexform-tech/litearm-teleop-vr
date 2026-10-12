# 快速上手

VR 遥操 LiteArm 七轴臂 + LiteGrip 夹爪。**下面所有命令都不会碰真机**：一个跑
`mock_components` 虚拟硬件，一个跑「假固件 + 真协议链路」。真机路径不在本仓交付范围内。

> 语言约定：代码注释与英文 `README.md` 为英文；本文件与 `README.zh.md`、各包的
> `README.zh.md` 是中文文档。两边内容不对应时以英文为准。

---

## 1. 30 秒上手

```bash
# ① 先叠臂的工作区（本仓依赖 litearm_ros2_control / litegrip_ros2_control / 臂描述）
source /opt/ros/humble/setup.bash
source ~/wkspace/ros2_ws/install/setup.bash

# ② 构建（第一次约 30 秒）
cd ~/github/vr_remote
colcon build --symlink-install
source install/setup.bash

# ③ 起虚拟栈（不起 RViz，先确认链路）
ros2 launch litearm_vr_teleop teleop_virtual.launch.xml use_rviz:=false
```

看到这三行就对了：

```text
[vr_teleop]: VR teleop ready: ... planning backend 'plugin'
[headset_link]: headset link serving on ws://0.0.0.0:8765
[spawner]: Configured and activated joint_state_broadcaster / vr_forward_controller / gripper_controller
```

## 2. 确认三个控制器真的活着

本机**没有装 `ros2controlcli`**（`ros2 control` 不是可用子命令），所以用服务查：

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1   # 本栈锁域；不设同两个变量什么都看不到
ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers "{}" | tr ',' '\n' | grep -E "name=|state="
```

期望：`joint_state_broadcaster` / `vr_forward_controller` / `gripper_controller` 三个都是
`state='active'`。

再看一眼命令通道只有**一个**订阅者（一个以上说明命令被写了两遍）：

```bash
ros2 topic info -v /vr_forward_controller/commands | grep -E "Type:|Subscription count"
# 期望 Type: std_msgs/msg/Float64MultiArray · Subscription count: 1
```

## 3. 接头显

头显连 `ws://<本机 IP>:8765`。换端口用 `headset_port:=8876`。

> ⚠️ **本机现在就有一个 8765 占用者**：`vr_control` 的 `ws_echo_server.py`
> （`~/github/vr_control/metavr/scripts/ws_echo_server.py`）。两边默认端口相同，
> 同时跑时本栈的 `headset_link` 会**报清原因并退出**（不会静默假装在跑）：
> `headset link could not serve on 0.0.0.0:8765: [Errno 98] address already in use`。
> 二者选一：停掉老的，或给本栈换端口。
>
> 想知道 8765 现在归谁：`ss -tlnp | grep 8765`

### 操作方式

| 动作 | 结果 |
|---|---|
| 按下手柄 grip | 先**同速回到遥操起点姿态**（`grasp_ready`），到位后开始跟随。各轴同速 ⇒ 行程小的轴先到先停（SDK 的 `movej` 语义，不是 `movej_sync`） |
| 松开 grip | 就地持位（写实测位置 + 速度 0，这是本机固件唯一真正的「停住」姿势） |
| 扣扳机 | 夹爪闭合 |
| 松开扳机 | 夹爪张开（按住不足 0.5 s 会被忽略，避免「还没夹住就松开」） |

外部停车 / 恢复：

```bash
ros2 service call /vr_teleop/stop  std_srvs/srv/Trigger   # 冻结，必须显式 start
ros2 service call /vr_teleop/start std_srvs/srv/Trigger
```

另外还有一条**头显断连看门狗**：0.5 s（`headset_timeout_s`）收不到帧就持位，并且
下次按 grip 会**重新锚定**（头显重连后的位姿与断连前不连续，不能当成连续的）。

## 4. 没有头显，怎么自测

直接往 WebSocket 灌合成帧即可（这条路我实测过：能完整驱动回起点 → 跟随 → 夹爪）。需要
`websockets` 模块，用有它的解释器跑：

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
        await hold(10.0, origin, True, 0.0)            # 按住 grip：臂回起点
        for i in range(61):                            # 手柄向自己右侧推 12 cm
            await ws.send(frame(i / 60.0 * 0.12, 1.0, 0.0, True, 0.0))
            await asyncio.sleep(1 / 30)
        await hold(3.0, (0.12, 1.0, 0.0), True, 0.0)   # 停住，等它收敛
        await hold(2.0, (0.12, 1.0, 0.0), True, 0.9)   # 扳机：夹爪闭合
        await hold(3.0, (0.12, 1.0, 0.0), True, 0.0)   # 松扳机：张开
        await hold(2.0, (0.12, 1.0, 0.0), False, 0.0)  # 松 grip：持位

asyncio.run(main())
EOF

# 有 websockets 的解释器（本机 = 项目的 venv；系统 python3 没有这个模块）
# 参数 = 你给 launch 的 headset_port（默认 8765）
/home/yudao/venvs/metavr-arm/bin/python /tmp/vr_fake_hand.py 8765
```

一边跑一边看关节角 —— **必须按名字取**，见下面的警告：

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

> ⚠️ **`/joint_states` 的数组顺序不是 `joint1..joint7`，而且每次重启都可能变。**
> 本机实测两次同样的 launch，得到的是同一个集合、不同的顺序：
> `gripper_opening_joint, joint2, joint3, joint5, joint6, joint1, joint4, joint7`
> 与 `joint2, joint3, joint5, joint6, joint1, joint4, joint7, gripper_opening_joint`。
> 顺序由资源管理器枚举接口的次序决定，广播器的 `joints` 参数只筛「发哪些」、管不了
> 「按什么顺序发」。**所以任何按 `position[i]` 取值的消费者（比如按 index 存
> `observation.state[8]` 的数据集）都会拿到一组「看起来很像数据」的乱序关节角**，
> 而且两次运行还不一样。一律按名字取。

判据（`teleop_virtual` 从全 0 起步；`teleop_sim` 的假固件从另一个姿态起步，所以看
「按下 grip 之后」的值）：

- 按住 grip 约 10 秒后 → `joint1..7` 应≈ `[0, -0.6458, 0, -1.9897, 0, -0.8378, 0]`（起点姿态）
- 推那 12 cm 期间 → `joint1` 变正、`joint3`/`joint5` 变负（手柄右移 = 基座 +x）
- 扣扳机 → `gripper_opening_joint` 变 `0.0`；松扳机 → `0.07479`

## 5. 要看可视化

```bash
ros2 launch litearm_vr_teleop teleop_virtual.launch.xml      # 默认带 RViz
```

RViz 里只有 `RobotModel` + `TF` + `Grid` + `Axes`（固定系 `base_link`）。**故意没有**
MoveIt 的 MotionPlanning 面板：本栈的规划在 `vr_teleop_node` 里，不起 `move_group`。

## 6. 仿真（假固件）怎么起

```bash
ros2 launch litearm_vr_teleop teleop_sim.launch.xml
```

它起真的固件守护进程，只是把固件换成 pty 上的假固件：协议、帧格式、100 Hz 状态上报
都是真的，串口与 CAN 一个都不开。**它能回答的事和虚拟栈不同**：

- 固件是否接受这条指令流（它有 100 ms 看门狗，指令流必须连续）
- 命令帧的 `position` 与 `velocity` 写没写对 —— 只写位置的 forward 控制器在这条路上
  会「收指令但一动不动」（固件把 velocity 当位置参考的斜率上限，0 就是冻结）
- 控制器周期与上报率是否匹配

**别用它整定增益**：假固件是一阶运动学模型，其自身文档明确说不能用来读动力学。实测
现象：臂**确实会动、方向也对**，但末端只走出指令位移的约四分之一 —— 那是一阶滞后被
「目标每帧按实测位姿重新基准」的速度式映射放大的结果，不是指令通道的问题（同一份代码
在 mock 上是 1.04 倍）。

## 7. 拍不到时按现象查

| 现象 | 先看什么 |
|---|---|
| 三个控制器没起全 | 启动日志里的 `spawner` 行；`Failed loading controller` 后紧跟的 ERROR 行才是真原因 |
| `vr_forward_controller` 报 `parameter 'joints' is empty` | 控制器节点没拿到自己的参数。检查 `spawner` 有没有 `--param-file`，以及 `ros2_control_node` 是不是被显式写了 `name=`（会注入 `__node:=` 重映射把控制器节点也改名，于是读到管理器的参数） |
| 管理器整个退出，日志里 `was not find in the interface name` | 命令接口名没拼成 `关节/接口`。本机 ros2_control 2.54.2 的 forward 基类返回裸名，必须自己覆盖 `command_interface_configuration()` |
| sim 栈里 `LitearmSystem` 报「打开共享内存失败」 | 控制器比守护进程起得早。本栈用 2 秒延时串联（`<timer period="2.0">`）；共享内存段名是 `/litearm_vr_hw` |
| 臂收指令但一动不动 | 命令帧的 velocity 全 0。检查 topic 上有没有发布者、以及控制器的 `max_velocity` 是不是被设成 0 |
| 夹爪不动（尤其 sim 栈） | 夹爪驱动是 fail-closed：`litegrip_max_feedback_velocity_rad_s` 没给就拒发任何运动帧。虚拟/假固件下 launch 会传一个推导值；真机必须实测标定 |
| 头显连上了但臂不动 | `headset_link` 每 10 s 会报一行「no headset is connected / last frame N s ago」；再确认 grip 真按下了（要上升沿，且 `tracked` 不能是 false） |
| 按 index 读 `/joint_states` 得到的关节角「像数据但不对」 | 顺序不是 `joint1..7`，也不是固定的。按名字取（见 §4）；这个坑不会报错，也不会在两次运行之间保持一致 |
| RViz 起来了但看不到机器人 | `RobotModel` 靠 `/robot_description`（Transient Local 锁存）；确认 `robot_state_publisher` 在跑 |
| 构建期报 `package 'litearm' not found`（xacro） | 当前环境里没有描述包。先 `source ~/wkspace/ros2_ws/install/setup.bash`（那儿的描述包叫 `litearm`）。**手工把 `share/` 拷进 install 树不算安装**，它没进 ament index |
| 构建期报 `needs an arm description that declares joint1..joint7 and ee_link` | 装的是这一臂的**另一代**描述（`Link1..Link7` / `ee_frame_link`）。报错会把找到的候选和各自原因列出来。需要 `joint1..7`+`ee_link` 那一代（`~/wkspace/ros2_ws` 里的 `litearm` 就是） |
| 启动横幅里 `arm description:` 不是你以为的那个包 | 用 `arm_description_package:=<包名>` 指定 |
| 终端里 `ros2 topic list` 是空的 | 本栈锁 `ROS_DOMAIN_ID=42` + `ROS_LOCALHOST_ONLY=1`，你的终端要 export 同样两个变量（launch 横幅里会打印这两行） |

## 8. 收工

Ctrl-C 那个 launch 即可（它在自己的进程组里）。核实没有残留：

```bash
pgrep -af 'vr_teleop_nod[e]|ros2_control_nod[e]|headset_lin[k]|rviz[2]|litearm_hw_daemo[n]'
ls /dev/shm/ | grep litearm          # 只跑虚拟栈的话应为空
```

`teleop_sim` 用的共享内存段是 `/litearm_vr_hw`（**故意**与臂栈默认的 `/litearm_hw`
分开，免得本栈接到一个正在跑的臂栈的段上去）。异常退出后它可能留在 `/dev/shm`，删掉即可。
