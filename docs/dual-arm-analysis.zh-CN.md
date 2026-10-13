# 双臂遥操改造分析：把「单臂 ↔ 双臂」的切换成本压到一个 launch 参数

这份文档回答一个问题：**在这个仓库现在的实现上增加第二条机械臂，怎样改才能让单臂/双臂的切换
只改配置、不改代码。** 它盘点了现状（哪些已经是参数、哪些写死了），给出推荐方案与逐条改动点，
列出已知的坑，并给出落地顺序。读者是准备做这件事的人。

> 说明：本文是决策用的工作文档，目前只有中文；英文对照版是后续动作。

---

## 1. 结论先行

**推荐方案 A：一条臂一个进程（`vr_teleop_node` 实例）、一条臂一个 forward 控制器、一条臂一份参数
文件，全部落在各自的命名空间里；单双臂的差异只存在于 launch 的一个参数和那份参数文件。**

它之所以能做到"切换几乎零改写"，是因为这个仓库已经有三个关键前提：

1. **`/joint_states` 是按名字取的**（`onJointState` 对每个关节名做 `std::find`），所以两条臂可以
   共享一条全局的 `/joint_states`，各自只挑自己的 7 个关节；
2. **`VrControllerFrame.msg` 里预留了 `side` 字段**，且 `headset_link.py` 已经用
   `HAND_TO_SIDE` 把左右手都发出来 —— 传输层已经是双臂就绪的，不需要新消息、不需要新节点；
3. **节点几乎全部行为都已经参数化**：话题、关节名、起点姿态、规划组、基准/末端坐标系、TCP 偏移、
   求解器与阻尼、滤波与限幅、夹爪动作名，全部来自参数（见 §2 的表）。

**方案 B：单进程内维护 N 条臂的状态**（把 `mapper_/planner_/cartesian_target_/phase_` 变成每臂一份）。
只有在需要**跨臂协同运动**（两手同时约束同一个目标，或双臂互不碰撞的规划）时才值得做；纯粹的
"两条臂各自跟随自己那只手柄"用不着，而它的改动面比 A 大一个数量级。

---

## 2. 现状盘点

| 项 | 现状 | 双臂需要 | 改动量 |
|---|---|---|---|
| 手柄帧消息 | `VrControllerFrame` 带 `side`（"left"/"right"），注释里写明"为第二条臂预留" | 直接复用 | **零** |
| 头显链路 | `headset_link.py` 按 `HAND_TO_SIDE` 发布左右手，各自带 `side` | 直接复用（可考虑一条臂一个链路实例，或共用一个） | **零** |
| 话题名 | `frame_topic` / `joint_state_topic` / `command_topic` 都是参数 | 每臂一套（放命名空间里最省事） | 配置 |
| 关节名 | `joint_names` 参数；状态按名字查找 | 每臂 7 个名字 | 配置 |
| 起点姿态 | `start_pose` 参数 | 每臂一份（左臂通常镜像） | 配置 |
| 规划组 / 基座 / 末端 | `group_name` / `base_frame` / `tip_link` 参数 | 每臂一个组 | 配置 + MoveIt |
| 求解器与阻尼 | `ik_backend`、`dls_*`、`damping_singular_value` 参数 | 每臂一份即可 | 配置 |
| 映射标定 | `control_frame`、`base_link_rotation_signs`、`tool_rotation_signs`、`tcp_offset`、`rotation_mode` 参数 | **每臂各一套**（安装朝向不同就必须分别标定） | 配置 |
| 映射的坐标系搬移 | `singleArmUprightMapping()` 里的 `axis_remap` / `translation_remap` 由节点参数给定 | 左臂若镜像安装，必须给不同的搬移矩阵 | 配置（需要新增可覆盖项，见 P7） |
| 手柄左右手选择 | **写死**：`onFrame` 里 `if (frame->side != "right")` 直接丢弃 | 改成参数 `side` | **小改（约 10 行）** |
| forward 控制器 | 一个实例、一个 `joints` 列表、一个命令话题 | 每臂一个实例（`vr_forward_controller_left/right`） | 配置 |
| 夹爪控制器 | 一个实例 + 一个动作名参数 | 每臂一个实例 + 每臂一个动作名 | 配置 |
| 硬件组件 | 一条臂的 USB 直连组件（一个串口）+ 一个夹爪 | 每臂一个串口组件 | URDF + 硬件 |
| URDF / SRDF / MoveIt | 单臂单夹爪、一个 group `litearm_arm`、一份 `kinematics.yaml` | 两个臂实例、两个 group、两份求解器条目 | **主要工作量** |
| 一致性测试 | 比较"节点那份求解器配置"与 MoveIt 那份 | 每臂一份比较 | 小改 |
| 目标可视化 `vr_target_viz` | 读帧、发布一个 tf frame + Marker | 每臂一个实例（各自命名空间） | 配置 |

---

## 3. 改动点逐条（方案 A）

### P1 手柄侧别变成参数（唯一的代码改动）

`src/vr_teleop_node.cpp` 的 `onFrame()` 现在丢弃 `side != "right"` 的帧。改成读参数
`side`（默认 `"right"`，保持单臂行为不变），并把那条 `WARN_ONCE` 改成"忽略另一只手"。约 10 行。

### P2 每臂一份参数文件

`config/vr_teleop.yaml` 保持单臂默认不动（单臂路径必须零改动），新增
`config/vr_teleop_left.yaml` / `vr_teleop_right.yaml`（或一份 `vr_teleop_dual.yaml` 用命名空间区分），
差异集中在：`side`、`joint_names`、`command_topic`、`group_name`、`base_frame`、`tip_link`、
`start_pose`、`tcp_offset`、两组 `*_rotation_signs`、`gripper_action`。

### P3 launch 里的臂数开关

`launch/teleop_common.py` 已经有"这次起哪些控制器"的列表函数；把它参数化为按臂生成，
入口加一个 `arms:=1|2`（或 `arm_layout:=single|dual`）。单臂时生成的结构与今天**逐字相同**，
双臂时多一组节点 + 一组 spawner 条目。这是"切换成本 = 一个参数"的落点。

### P4 控制器的两份声明

`config/vr_controllers.yaml` 里把 `vr_forward_controller` / `gripper_controller` 各复制一份，
改 `joints` 与命令话题；双臂时 spawner 列表里出现四个控制器。

### P5 URDF / SRDF / MoveIt（真正的工作量）

- `litearm_vr.urdf.xacro`：把"一条臂 + 一个夹爪"抽成可实例化的宏，用 `prefix` 参数实例化两次；
- `litearm_vr.srdf.xacro`：两个 group（`litearm_arm_left/right`）与各自的 `group_state`；
- `config/kinematics.yaml`：两个 group 各一条求解器条目（KDL 或 SNS，与节点那份保持一致 —— 一致性
  测试会盯着这一点）；
- `litearm_vr_moveit_config` 的 `arm_description_package` 探测与 URDF 校验逻辑要能接受双臂。

### P6 硬件

ros2_control 的每条臂各自一个 USB 组件（各占一个串口），夹爪同理；`dry_run` 语义不变
（双臂时"仿真的那条臂"仍然是 mock）。安全上要明确：**停一条臂不等于停另一条**。

### P7 每臂标定（最容易踩的坑）

遥操映射把"手柄坐标系"搬到"机器人坐标系"，靠的是 `axis_remap` / `translation_remap` 加两组
`*_rotation_signs`。左臂如果是**镜像安装**，这两个矩阵与符号都不能与右臂共用 —— 需要在参数里
把 `axis_remap` / `translation_remap` 暴露出来（今天它们由 `singleArmUprightMapping()` 给死），
然后每臂各标一次：`tcp_offset`、`start_pose`、两组符号、搬移矩阵。

### P8 命名空间

每臂一个命名空间（`/left`、`/right`）是最省心的做法：话题、服务（`~/stop`、`~/start`）、参数
全自动隔离，launch 里不需要手写每个名字。`/joint_states` 与 tf 保持全局。

### P9 测试

- `test_kinematics_consistency.py`：按臂各比一次（两臂的 group 名不同）；
- `test_teleop_math.cpp`：加一条"左臂镜像搬移"的映射用例（同一个手柄动作，两条臂的解互为镜像）；
- `test_launch_arguments.py`：`arms:=1` 与 `arms:=2` 各断言一次生成的控制器集合。

---

## 4. 为什么切换成本能压到"一个参数"

三条不变式，改代码时守住它们，双臂就不会渗进单臂路径：

1. **代码里不出现 "left"/"right"**。侧别、组名、话题、标定全部来自参数；只有 P1 那一个参数读入点。
2. **一切按名字取，不按序号**（关节、控制器、组）。`/joint_states` 的顺序不稳定这一点已经在
   quickstart 里写明；双臂会让这个坑更贵，因为顺序里还会混进另一条臂的关节。
3. **单臂是默认、且是"少一个实例"而不是"多一个分支"**。`arms:=1` 走的是同一段生成代码，
   生成一个实例；这样单臂路径不会因为双臂而腐化。

---

## 5. 已知的坑

| 坑 | 说明 |
|---|---|
| 求解器的单线程契约 | `TeleopPlanner` 用共享的 scratch robot state，文档里明确"单线程使用"。每臂一个实例即可，但**不要**让两条臂共用同一个 planner 对象；求解线程也是每臂一条（或串行化） |
| 左臂镜像安装 | 映射的搬移矩阵与符号必须分别标定；把右臂的值抄给左臂，表现是"左臂对手柄的响应方向有一半是反的" |
| 夹爪 fail-closed | `litegrip_max_feedback_velocity_rad_s` 没给就拒发运动帧 —— 双臂时每臂都要给 |
| 单臂路径退化 | 左臂参数文件缺失时，launch 必须干净地退化到单臂（而不是起一个空臂） |
| 命名冲突 | 两个节点的服务名、参数名完全相同，只有命名空间能区分；缺了命名空间会出现"两个节点抢同一个参数"这类难查的错 |
| 安全 | 两条臂同时运动时的急停语义要重新定义（一个 `~/stop` 只停自己） |
| 一致性测试 | 双臂后 MoveIt 的 `kinematics.yaml` 与节点那份**两份都**要对上，测试要按臂展开 |

---

## 6. 什么时候该转向方案 B

出现下列任一需求时，才值得把状态搬进单进程的多臂结构：

- 双臂**协同**运动（夹持同一物体、相对位姿约束）；
- 双臂**互碰撞**规避（需要共享一个场景模型）；
- 需要双臂共用一个"握持状态机"（比如交接物体）。

在那之前，方案 A 的进程间边界反而是优点：一条臂崩了不影响另一条，测试与回滚都以臂为单位。

---

## 7. 建议的落地顺序（可直接切成 PR）

1. **P1**：`side` 参数 + 单臂默认不变（代码改动最小，先合）；
2. **P2/P3/P4**：参数文件按臂拆分 + launch 的 `arms` 开关 + 控制器第二份声明（此时 `arms:=2` 能起
   两条"仿真臂"，但 MoveIt 组还没有）；
3. **P5**：URDF/SRDF/kinematics 的双臂化（`arms:=2` 首次真正可用）；
4. **P6**：硬件与安全文档；
5. **P7/P9**：左臂标定 + 测试按臂展开。

每一步都能单独验证：1 用现有测试；2 用 `arms:=2` 起仿真看四个控制器是否 active；3 用 MoveIt 的
组列表与 IK 残差；5 用"同一个手柄动作左右臂互为镜像"这条断言。
