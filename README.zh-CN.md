# litearm-teleop-vr（中文）

LiteArm 七轴机械臂 + LiteGrip 夹爪的 VR 遥操，跑在 ros2_control 上。

两个包：

| 包 | 是什么 |
|---|---|
| `litearm_vr_teleop` | 遥操栈本体：头显通信、遥操节点、以及驱动硬件的 ros2_control forward 控制器。**原理与取舍都在它自己的 `README.zh.md` 里**，本文件只做索引。 |
| `litearm_vr_moveit_config` | 臂+夹爪合体的 MoveIt 配置：URDF、SRDF、kinematics、限位、控制器清单、RViz 视图。 |

> **语言约定**：代码注释与英文 `README.md` 是英文；本文件与 `docs/QUICKSTART.zh.md`
> 是中文文档。两边不一致时以英文为准。

---

## 快速上手

看 [`docs/QUICKSTART.zh.md`](docs/QUICKSTART.zh.md)（5 分钟：构建 → 起栈 → 接头显 → 没有头显
也能自测 → 排障清单）。

## 构建

本仓依赖 `litearm_ros2_control`、`litegrip_ros2_control` 与臂的描述包，它们住在臂的工作区
里，所以要先把那个工作区叠上来：

```bash
source /opt/ros/humble/setup.bash
source <臂工作区>/install/setup.bash      # litearm_ros2_control、litegrip_ros2_control、描述包

cd <colcon 工作区>
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch litearm_vr_teleop teleop_virtual.launch.xml   # 虚拟硬件（mock），什么真东西都不碰
ros2 launch litearm_vr_teleop teleop_sim.launch.xml       # 假固件 + 真协议链路
```

**两个 launch 不能互相替代**，它们各自回答不同的问题：

- `teleop_virtual` 把被控对象换成 `mock_components/GenericSystem`：没有固件、没有守护
  进程、没有共享内存、没有协议，臂「即时到位」。用它看**算术**：轴方向对不对、回不回得到
  起点姿态、夹爪开合、松手会不会漂。
- `teleop_sim` 起真的固件守护进程，只是固件是 pty 上的假固件：真协议、真帧、真 100 Hz
  上报，串口与 CAN 一个都不开。用它看**只有真链路才存在的行为**：固件收不收这条指令流、
  指令流连不连续（固件有 100 ms 看门狗）、以及 `position` 与 `velocity` 是不是都写对了
  —— **只写位置的 forward 控制器在这条路上「收指令但一动不动」**。

两者都**不可能**碰真机：dry run 是硬编码的（不是可传参数），共享内存段也单独命名
（`/litearm_vr_hw`，不是臂栈的 `/litearm_hw`），所以本栈没法接到一个正在跑的臂栈的段上。

一个数据流概览：

```text
Quest 3 WebXR 页面（~72 Hz，ws 明文 JSON）
   → headset_link.py（只做传输：解析、发布、计数）
   → /vr_controller_frame
   → vr_teleop_node（C++：遥操语义 + IK 规划 + 100 Hz 指令流）
   → /vr_forward_controller/commands（7 个位置）
   → VrForwardController（同帧写 position + velocity）
   → LitearmSystem / mock（ros2_control 硬件组件）
   → 固件守护进程 → USB CDC → 臂
```

## 为什么是 forward 控制器，而且必须同时写速度

本机固件把指令帧里的 velocity 当**位置参考的斜坡上限**。只认领 `position` 的控制器会让
velocity 接口停在硬件初始值 0 上 —— 于是参考被冻结，**臂收下指令但一步不动，且不报任何错**。
所以 `VrForwardController` 派生自 `forward_command_controller::ForwardControllersBase`
（它确实是一个 forward 控制器：指令流进、硬件接口出、没有轨迹也没有目标），但覆盖
`update()` 把两条接口一起写。

细节与另外两处必须覆盖的地方（本机 ros2_control 2.54.2 的两条坑）见包内
[`README.zh.md`](litearm_vr_teleop/README.zh.md)。

## 测试

```bash
colcon test
colcon test-result
```

C++ 侧覆盖增量映射、关节滤波、点到点轮廓（在理想被控对象上）；Python 侧覆盖帧解析的容错
与回调交接。测试证明不了「栈能跑」——那是两个 launch 的事；测试证明的是「符号写反了/增益
算错了，在到电机之前就被抓住」。

## 来源

头显通信与遥操映射移植自参考实现 `vr_control`（`metavr/scripts/ws_echo_server.py`、
`metavr/src/litearm_teleop/teleop_sim.py`，以及跟随服务的限速）。移植过来的数字都是**实测
值**，用到的地方都注明了来源；它们不是在本仓推导出来的。凡本栈刻意与参考实现不同的地方，
注释里都写了差异与理由（最典型的是关节低通滤波放在「每周期步长」而不是「绝对目标」上，
因为放在绝对目标上会让跟随增益变成滤波系数 —— 实测 12 cm 手柄位移只走出 8 cm）。
