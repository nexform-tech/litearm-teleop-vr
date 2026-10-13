# litearm_vr_moveit_config（中文）

「七轴 LiteArm + LiteGrip 夹爪」合体机器人的**描述与求解器配置**。

本包**不做规划**：不起 `move_group`，没有 OMPL 流水线、没有下发控制器表、也没有规划用关节限位
—— 遥操栈要的是三样东西：一份 `robot_description`、一份 `robot_description_semantic`（规划组
就定义在里面，命令数组的关节顺序也钉在它上），以及 `plugin` 后端要用的求解器调参。名字里的
`moveit_config` 是历史：留下的 `srdf/` 与 `config/kinematics.yaml` 都是 MoveIt 的文件格式，
而 `urdf/`、`rviz/` 与描述包探测是这个包自己的活。要跑 MoveIt 规划的话，臂那份
（`litearm_moveit_config`）才是入口，它自带 OMPL 与 `move_group` 需要的全套配置。

> 代码注释为英文；本文件是中文文档，与英文侧不一致时以注释为准。
> 使用者是 `litearm_vr_teleop`，快速上手见仓库根的 [`QUICKSTART.zh.md`](../docs/QUICKSTART.zh.md)。

## 文件构成

| 路径 | 内容 |
|---|---|
| `urdf/litearm_vr.urdf.xacro` | 合体机器人描述：臂 + 夹爪几何 + **ros2_control 硬件块**（见下） |
| `srdf/litearm_vr.srdf.xacro` | 语义描述：`litearm_arm` 组 + `gripper` 组 + 命名位形 + 碰撞矩阵 |
| `config/kinematics.yaml` | 求解器与它的调参（SNS-IK） |
| `rviz/litearm_vr.rviz` | 只有 `RobotModel` + `TF` + `Grid` + `Axes`，固定系 `base_link` |

每个文件**谁在用**见下面「与 `litearm_vr_teleop` 的接口」一节 —— 这张表只回答"仓库里有什么"。

## 硬件块的开关：`use_mock_hardware`

描述文件里的 `<ros2_control>` 块是**这个包唯一会变的东西**，由 xacro 参数
`use_mock_hardware` 决定：

| 取值 | 结果 |
|---|---|
| `false`（默认） | 引入 `litearm_ros2_control` 与 `litegrip_ros2_control` 两个真硬件块 ⇒ **两个** `<ros2_control>`（`LitearmSystem` + `LitegripSystem`） |
| `true` | 换成**一个** `VrMockSystem`，插件 = `mock_components/GenericSystem`，无 `<param>` ⇒ 没有固件、没有守护进程、没有共享内存、没有协议 |

之所以把开关放在描述里而不是参数里：**ros2_control 的硬件组件本身就是描述的一部分**，
哪块被加载是「用了哪份文件」决定的，不是运行期参数决定的。

mock 块导出的接口面刻意与真硬件保持一致（臂：position+velocity 命令 / position+velocity
状态；夹爪：position 命令 / position+velocity+effort 状态）。**接口缺一条不是告警而是拒绝
激活**，所以两边必须对上，否则虚拟栈和真栈跑的是不同代码路径。

自己展开验证：

```bash
source /opt/ros/humble/setup.bash
source ~/wkspace/ros2_ws/install/setup.bash
xacro urdf/litearm_vr.urdf.xacro use_mock_hardware:=false | grep -c "<ros2_control"   # 2
xacro urdf/litearm_vr.urdf.xacro use_mock_hardware:=true  | grep -c "<ros2_control"   # 1
xacro srdf/litearm_vr.srdf.xacro | grep -E "group_state|end_effector"
```

## 臂描述包：只有一个候选，代际由 URDF 判定

本包的 mock 硬件模式需要**不含任何 `<ros2_control>` 块**的臂描述（它要替换掉硬件，不能把真
硬件块一起继承进来）。真硬件模式不走这条路：它 include `litearm_ros2_control` 的入口，由对方
去引描述。**只有 mock 模式需要点名描述包**，而这个名字是**工作区的属性**，不是本包的属性：

| 描述包名 | 描述文件 |
|---|---|
| `litearm_description` | `urdf/litearm_description.urdf` |

描述文件按 `<包名>.urdf` 命名，所以一个名字就够拼路径。**这个名字不靠猜**：`CMakeLists`
在 configure 期按 **ament index** 找（与 `$(find)` 运行时同一套机制——这正是唯一要紧的判据），
把结果写进 `config/arm_description_package.txt` 供 launch 读取，launch 再作为
`arm_description_xacro` 传给 xacro。

为什么不在 xacro 里拼：`$(find ...)` 只吃字面量，**不替换它自己括号里的 `$(arg ...)` 或
`${...}`**（两种都试过，都报错）。launch 的 substitution 是可以嵌套的，所以拼接放在 launch 侧。

**比名字更硬的一条约束：代际。** 本包（以及 `litearm_ros2_control` 的硬件块、本工作区
`litearm_moveit_config` 的 SRDF）都按 `joint1..joint7` + `ee_link` 这套命名书写，也就是
`litearm_description` 这一代。已被取代的那份 CAD 导出（关节 `Joint1..Joint7`、link
`Link1..Link7`、法兰 `ee_frame_link`）**不是同一台臂的改名**：7 个关节原点 0 个一致、总质量
3.5305 kg vs 2.7911 kg。用它拼出来的 URDF 会被 urdfdom 拒绝：

```text
parent link [ee_link] of joint [gripper_mount_joint] not found
```

所以 configure 期**真的读它的 URDF**，确认它声明了 `joint1` 与 `ee_link`；不符合就带原因
**FATAL_ERROR**，而不是等到 xacro 报「package not found」那种
看起来像安装坏了的错。要临时指定别的名字：

```bash
ros2 launch litearm_vr_teleop teleop_arm_gripper.launch.py arm_description_package:=<包名>
```

⚠️ **把 `share/` 目录手工拷进 install 树不会注册包**：`ament index` 里没有它，`$(find)` 照样
找不到；而它的 CMake config 可能还在，于是 `find_package` 会「找到」一个用不了的前缀
（报 `imported targets are referenced, but are missing`）。要装就 build，别拷。

## 两个必须同步的值

**① 遥操起点姿态 = SRDF 的 `grasp_ready`。**

```xml
<group_state name="grasp_ready" group="litearm_arm">
  joint1 0, joint2 -0.645772, joint3 0, joint4 -1.989675, joint5 0, joint6 -0.837758, joint7 0
</group_state>
```

SRDF 让 MoveIt 能按名字用它；`litearm_vr_teleop` 的 C++ 侧把同一组数**硬编码**了一份，好在
不需要规划器在场的情况下也能走到那个位姿。两处必须一起改 —— `test_kinematics_consistency.py`
会比对它们，漂开就测试失败。

为什么要 `grasp_ready` 而不是臂本体的 `ready`：前者是「抓取前先回的位姿」（也用于收尾回退），
后者是「本体舒展位」，两者可以各自演化。用哪个是遥操侧的决定，本包只负责把它命名出来。

**② 求解器调参。** `config/kinematics.yaml` 里的 `litearm_arm` 块与
`litearm_vr_teleop/config/vr_teleop.yaml` 里同名的五个值（`kinematics_solver` /
`solver_type` / `loop_period` / `eps` / `timeout`）是同一组数的两份拷贝，原因与那守卫测试见
遥操包的 `README.zh.md` 第五节。

一个坑：MoveIt 侧那份写的是 `eps: 1e-5`，而 YAML 把 `1e-5` 读成**字符串**（YAML 的浮点要求
小数点前有数字）；遥操侧那份写的是 `1.0e-5`，读成浮点，也才是插件要的（它按 double 读这个
参数，字符串会让它抛异常）。所以那个测试对这两个值做的是**数值比较**而不是文本比较。

## 几何与命名来源

- **臂**：来自 `litearm` 描述包（`litearm_ros2_control/urdf/litearm.urdf.xacro` 转出）。
- **夹爪**：几何按 `litearm_manipulation/urdf/litearm_gripper.urdf.xacro` 逐字复制。该文件重述
  `litegrip_urdf` 的 CAD 运动学，唯一原因是 **link 名冲突**（上游把夹爪本体也叫 `base_link`，与
  臂根同名），所以夹爪 link 统一加 `gripper_` 前缀，mesh 仍直接引用
  `package://litegrip_urdf/meshes/*.STL`，不复制网格。
- **去掉了相机子树**（`camera_body_link` / `camera_mount_joint` / `camera_collision`）：本栈没有
  相机。改动几何时若要参照上游文件，注意别把这块带回来。

命名位形除了 `grasp_ready` 还有 `zero` / `ready`（臂，来自 `litearm_moveit_config/config/litearm.srdf`）
与 `open` = 0.07479 / `closed` = 0.0（夹爪）。

**为什么夹爪全开是 0.07479 而不是 CAD 的 0.087**：驱动红线只允许开到 74.79 mm，而
`allow_stalling: true` 会把「位置停住」判成**成功** —— 所以够不到的目标**不报错也不挂，只是
静默少开 12.2 mm**。这个上限由臂侧测试从驱动包重算并断言。

## 碰撞检查

只禁用三类**结构性相邻**与一对共轴手指：

```text
ee_link ↔ gripper_base_link              固定连接
joint7 ↔ gripper_base_link               固定连接
gripper_base_link ↔ gripper_slider_link1/2   prismatic，零位时几何恰好相接
gripper_slider_link1 ↔ gripper_slider_link2  同一根导轨上的对开两指，闭合时内侧面按设计接触
```

其余组合**一律保留检查**：夹爪撞到臂或桌面是真实碰撞，必须能被看见。臂本体的碰撞矩阵由
`litearm_moveit_config` 用真实网格算出（`tools/compute_collision_matrix.py`），本包直接
`xacro:include` 它，不复制，所以那边改了会自动跟过来。

## 与 `litearm_vr_teleop` 的接口

| 本包提供 | 遥操侧怎么用 |
|---|---|
| `urdf/litearm_vr.urdf.xacro`（带 `use_mock_hardware` 开关） | launch 里 `xacro` 展开成 `robot_description`，喂给 `robot_state_publisher`、`ros2_control_node`、`vr_teleop_node`、RViz |
| `srdf/litearm_vr.srdf.xacro` | 展开成 `robot_description_semantic`，喂给 `vr_teleop_node`（建 MoveIt 模型必须有它，规划组就定义在里面）与 RViz |
| `config/kinematics.yaml` | `vr_teleop_node` 的 `plugin` 后端按类名加载求解器；调参数值另有一份在遥操侧 |
| `rviz/litearm_vr.rviz` | launch 的 `rviz2 -d` 参数 |

注意 **`robot_description` 与 `robot_description_semantic` 必须显式声明成 `str`**：launch 对
「没写类型的参数」会拿 YAML 去推断值，而 URDF/SRDF 是长文本，里面任何一行注释里出现 `:` 加空格，就会让
YAML 解析器抛错，报成「Failed to convert <整份 URDF>」。
