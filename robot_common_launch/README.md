# robot_common_launch

跨机器人共用的 launch 入口、xacro 映射，以及 **machine profile**（任意路径的 YAML，经 `robot_profile:=` 传入）加载与合并逻辑。

下游 launch（可视化、`controller_manager`、各控制器包）通过本包 API 生成 `robot_description` / planning URDF，并合并 profile 与 CLI 参数。

## 机制概览

```text
ros2 launch … robot_profile:=/path/to/profile.yaml  [+ CLI args]
        │
        ▼
 load_robot_profile / normalize_robot_profile
        │
        ▼
 build_xacro_mappings  或  build_visualization_xacro_mappings
        │
        ▼
 xacro → robot_description / planning URDF
```

实现入口（[`launch_arg_utils.py`](robot_common_launch/common/launch_arg_utils.py)）：

- `load_robot_profile` / `normalize_robot_profile`
- `build_xacro_mappings`（ros2_control / planning）
- `build_visualization_xacro_mappings`（纯 RViz，见 [`launch_utils.py`](robot_common_launch/common/launch_utils.py)）

任何 launch 只要声明并转发 `robot_profile`（例如 `create_robot_profile_launch_arguments()`），都会走同一套机制；**不依赖**某一部署仓库的交互脚本。

## 合并优先级

**Launch CLI > `robot_profile` YAML > xacro 默认值**

| YAML 段 | 随 `robot_profile:=` | 说明 |
|---------|----------------------|------|
| `platform` | 始终 | 底盘 / 双臂套件 / `variant` / `chassis_joints_movable` 等 → xacro |
| `defaults.end_effectors` | 仅 `use_profile_eef:=true`（默认） | 末端类型；可被 CLI 关掉以改用 `type` / `left_type` / `right_type` |
| `defaults.ft` | 始终 | 力传感器；**不受** `use_profile_eef` 影响 |
| `defaults.tcp_offset` | 始终 | 虚拟 tip 偏移；注入前做表达式求值 |
| `hardware` | 仅 `hardware:=real` / `real_usb` | 真机参数（串口、`arm_ctrl_mode`、`*_dyn_param` 等） |
| `control.patch` | 始终 | 深合并进 ros2_control 配置 |

### Profile schema（包约定）

```yaml
platform:
  chassis: <key>
  arms: <key>                 # 双臂套件（机型相关；对应 {key}_description 或带 _vN 后缀）
  variant: <key>
  chassis_joints_movable: "true"|"false"
  # Optional — remote chassis over ROS 2 / Zenoh (default off):
  # remote_chassis: "true"
  # remote_chassis_joint_states_topic: "/chassis/joint_states"
  # body_joint_states_topic: "/body/joint_states"
  # merged_joint_states_topic: "/joint_states"

defaults:
  end_effectors:
    type: <eef_key>            # 对称；或 left / right 分写
  ft:
    type: <ft_key>             # none | kwr75_485 | kwr75_usb；或 left / right
  tcp_offset:
    xyz: "0 0 0"               # metres；或 left_xyz / right_xyz
    rpy: "0 0 0"               # radians；或 left_rpy / right_rpy

hardware:
  arm_ctrl_mode: position
  # usb_* / *_dyn_param / …

control:
  patch:
    <controller_name>:
      ros__parameters: { … }
```

`end_effectors` / `ft` / `tcp_offset` 均支持对称键（`type` 或 `xyz`+`rpy`）展开到左右；缺侧回退对称值。

### `tcp_offset` 表达式

经 mappings 传入的字符串 **不会**被 xacro 二次求值。本包在写入 mappings 前预处理：

- 允许：`${PI/2}`、`pi/2`、`radians(90)`、纯数字
- 安全符号：`pi` / `PI` / `radians`
- 求值失败则报错，不静默写入 URDF

选用虚拟 tip 时，还需在 `control.patch`（或控制器参数）中设置 `left_ee_frame` / `right_ee_frame`（例如 `left_tcp_offset`）。

## ros2_control 配置合并（`load_robot_config`）

控制器参数由 [`robot_utils.load_robot_config`](robot_common_launch/common/robot_utils.py) 组装，OCS2 / `controller_manager` launch 都会走这里。

### 合并顺序

```text
common.yaml
  → <type>.yaml / ros2_controllers.yaml
  → {variant}.yaml          # opt-in：config/ros2_control/<variant>.yaml
  → {hardware}.yaml         # opt-in：config/ros2_control/<hardware>.yaml（如 isaac.yaml）
  → EEF compose（如有）
  → control.patch（profile）
```

- **variant** 来自 launch `variant:=` 或 profile `platform.variant`
- **hardware** 来自 launch `hardware:=`（如 `mock_components` / `gz` / `isaac` / `real`）
- 对应 YAML **仅当文件存在时**才深合并；没有文件则为空操作（不影响未声明 overlay 的机器人）
- 后写入覆盖先写入的同名键（list / 标量整段替换；dict 递归 merge）
- 任一 overlay / compose / patch 生效时会写出临时 merged YAML 供 `ros2_control_node` 使用

与上文 profile 段里的 `hardware:`（真机串口、`arm_ctrl_mode` 等 → xacro）**不是同一层**：后者只在 `hardware:=real` / `real_usb` 时注入 xacro；此处 `{hardware}.yaml` 改的是 **控制器 claim / 参数**。

### `{hardware}.yaml` 典型用途（Isaac）

部分机器人在 Isaac 中不宜 claim velocity/effort（TopicBasedSystem 对 MIX 支持差）。可在该机器人包放置：

```text
<robot>_description/config/ros2_control/isaac.yaml
```

在 `hardware:=isaac` 时自动合并，例如把 `command_interfaces` / `state_interfaces` 收成仅 `position`。  
**URDF 导出的 IF 须与 claim 一致**（由该机器人自己的 `xacro/ros2_control/*.xacro` 按 `ros2_control_hardware_type` 分支处理）。

| 目标 | 做法 |
|------|------|
| Isaac 用 position-only | 提供 `isaac.yaml` 改 claim + xacro 在 isaac 下只导出 position（例：`arx_lift2s`） |
| Isaac 仍用 MIX / 原配置 | **不要**放会覆盖 IF 的 `isaac.yaml`；URDF 继续导出完整 IF |
| 其它硬件特化 | 同理可放 `gz.yaml` / `real.yaml` 等，文件名 = launch `hardware` 值 |

Variant 与 hardware 可同时生效（先 variant，后 hardware）；hardware 侧通常只改接口能力，不替代机型变体字段。

示例（Lift2S）：

```bash
ros2 launch ocs2_arm_controller full_body.launch.py \
  robot:=arx_lift2s hardware:=isaac
# → 合并 arx_lift2s_description/config/ros2_control/isaac.yaml
```

## Launch 一等参数

由 `create_common_launch_arguments` / `create_robot_profile_launch_arguments` 等声明（具体 launch 可只挂子集）。

| 参数 | 含义 |
|------|------|
| `robot_profile` | profile YAML 绝对/相对路径 |
| `use_profile_eef` | 是否应用 `defaults.end_effectors`（默认 `true`） |
| `type` | 对称末端，或臂拓扑 `left`/`right`/`dual`（拓扑不会展开成 `left_type`/`right_type`） |
| `left_type` / `right_type` | 左右末端键 |
| `ft` / `left_ft` / `right_ft` | 力传感器（覆盖 profile `defaults.ft`） |
| `tcp_offset_xyz` / `tcp_offset_rpy` | 对称 TCP 偏移 |
| `left_tcp_offset_*` / `right_tcp_offset_*` | 分侧 TCP 偏移 |
| `chassis` / `arms` / `variant` / `chassis_joints_movable` / `remote_chassis` | 平台 xacro 与远程底盘 mux，见下节；`variant` 同时可触发 `{variant}.yaml` 控制器 overlay |
| `hardware` | 仿真/真机插件（→ xacro `ros2_control_hardware_type`）；并可触发 `{hardware}.yaml` overlay |
| `collider` / `skin` / `direction` | 常用模型 xacro |

### 平台槽位：`chassis` / `arms` / `variant`

由 `create_platform_launch_arguments()` 声明，供人形 / 移动底盘 xacro 使用（独立机械臂 launch 不挂这组）。`humanoid.launch.py`、`component.launch.py` 以及 OCS2 `full_body` / `split_body` / `demo` 已接入。

解析顺序（`resolve_robot_arms` / `resolve_robot_variant`）：**CLI > profile `platform.*` > 该机 `xacro/robot.xacro` 的 default**。空字符串表示不向 xacro 传该参数。

| 参数 | 作用 | 典型键 |
|------|------|--------|
| `chassis` | 底盘型号 | 机型相关 |
| `arms` | 人形双臂套件 | 机型相关；键名应对应 `{key}_description`，或带 `_v<digits>` 代际后缀（落到去掉后缀的包） |
| `variant` | 机型变体（外观、立柱、独立机械臂代际） | 机型相关。选臂的人形用 `arms`，不要把臂套件写进 `platform.variant` |
| `chassis_joints_movable` | 底盘关节是否可动 | `true` / `false` |

### 远程底盘（`remote_chassis`，可选，默认关闭）

用于**上半身 mock 运控 + 远端真机底盘**（ROS 2 / `rmw_zenoh_cpp` 跨机）。未配置时行为与现网完全一致，不影响其它机器人。

**启用条件**（任一即可）：

- `robot_profile` → `platform.remote_chassis: "true"`
- Launch：`remote_chassis:=true`

**Profile 示例**（联调 fa-w2 + 3588 底盘）：

```yaml
platform:
  chassis: linkhou_s2
  chassis_joints_movable: "true"   # 控制 URDF 轮系可动；规划 URDF 仍冻结
  remote_chassis: "true"
  remote_chassis_joint_states_topic: "/chassis/joint_states"
  body_joint_states_topic: "/body/joint_states"
  merged_joint_states_topic: "/joint_states"
```

**机制**（`create_controller_manager_nodes`）：

1. 本地 `joint_state_broadcaster` 经 remap 发布到 `/body/joint_states`（仅躯干/臂/手，不含轮系）。
2. 启动 `joint_state_mux`：订阅远端 `/chassis/joint_states` + 本地 `/body/joint_states`，按关节名合并后发布 `/joint_states`。
3. `robot_state_publisher` 仍读控制 URDF；需 `chassis_joints_movable:=true` 才能用合并后的轮系关节驱动 TF。
4. 启动 `static_transform_publisher`：`world → odom`（恒等），与底盘 `odom → base_link`（Zenoh）及 RSP 的 `base_footprint → base_link` 拼成 WBC 所需的 `world → base_footprint`。

**底盘侧为何不跑 RSP**：真机轮角/舵角走 `/chassis/joint_states`，里程走 `/chassis/odom` + `odom→base_link` TF；若 3588 也跑 RSP 会与主机抢同一 link 名的 TF。Jazzy 仍需要 URDF topic → 用 `robot_description_publisher`（无 TF）。

**适用 launch**：OCS2 `full_body.launch.py`、`split_body.launch.py`（均走 `create_controller_manager_nodes`）。`split_body` 仅 mock 双臂时通常 `planning_scope:=arms`，控制栈仍可用 `remote_chassis` 合并全身 JS。

**底盘侧约定**（非本包）：3588 上 `linkhou_s2_description` 以 `namespace:=chassis` 启动，发布 `/chassis/joint_states`、`/chassis/odom` 等；与上半身联调时建议 `publish_robot_state:=false`，避免与上半身 RSP 的 TF 冲突。

**验证顺序**：

```bash
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo world base_footprint   # WBC 所需
ros2 run tf2_ros tf2_echo odom base_link           # 来自 /chassis/odom TF
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `remote_chassis` | `false` | 启用 mux + JSB remap |
| `remote_chassis_joint_states_topic` | `/chassis/joint_states` | 远端底盘 JS |
| `body_joint_states_topic` | `/body/joint_states` | 本地 JSB 输出 |
| `merged_joint_states_topic` | `/joint_states` | RSP / RViz 订阅 |

`arms` **不会**触发 `config/ros2_control/{arms}.yaml` overlay（那是 `variant` 的职责）。分体规划若落到独立机械臂包（该包 xacro 不声明 `arms`），会丢掉人形的 `arms` 参数；若该包声明了 `variant`，则把同一键写入 `variant`。`planning_robot_for_arm_family` 用 ament 查找 `{key}_description`；找不到时去掉末尾 `_v<digits>` 再试。

```bash
ros2 launch robot_common_launch humanoid.launch.py robot:=fiveages_w1 arms:=ar5_ccs
ros2 launch robot_common_launch humanoid.launch.py robot:=fiveages_w2r arms:=ar5_srs
```

### 前缀逃逸口

| 前缀 | 用途 |
|------|------|
| `xacro_*` | 任意 xacro arg |
| `hardware_*` | 真机参数（仅 real / real_usb 路径注入） |

一等参数优先于同名的 `xacro_left_ft` / `xacro_left_tcp_offset_*` 等。

### 通用示例

```bash
# 任意 profile 路径
ros2 launch <pkg> <file>.launch.py \
  robot:=<robot_name> \
  robot_profile:=/path/to/machine_profile.yaml

# CLI 覆盖 FT / TCP（表达式）
ros2 launch robot_common_launch humanoid.launch.py robot:=<robot_name> \
  robot_profile:=/path/to/machine_profile.yaml \
  ft:=kwr75_485 \
  tcp_offset_rpy:="0 0 ${PI/2}"

# 不用 profile 末端，改用 CLI；platform / ft / tcp_offset 仍可来自 profile
ros2 launch <pkg> <file>.launch.py \
  robot_profile:=/path/to/machine_profile.yaml \
  use_profile_eef:=false \
  left_type:=rg75 right_type:=linkerhand_o7

# CLI 覆盖双臂套件（platform.arms）
ros2 launch robot_common_launch humanoid.launch.py robot:=<robot_name> \
  robot_profile:=/path/to/machine_profile.yaml \
  arms:=ar5_srs
```

## Launch 入口（节选）

```bash
colcon build --packages-up-to robot_common_launch --symlink-install
source install/setup.bash
```

| 场景 | Launch |
|------|--------|
| 通用可视化 | `visualize.launch.py` / `humanoid.launch.py` / `manipulator.launch.py` |
| 夹爪 / 灵巧手 | `gripper.launch.py` / `hand.launch.py` |
| 部件 | `component.launch.py` |
| 控制器管理 | `controller_manager.launch.py` |
| 导航 | `navigation*.launch.py` |

机器人专用用法与本机 profile 文件约定，见各部署工作区 / `*_description` 包文档。

## 相关代码

- [`robot_common_launch/common/launch_arg_utils.py`](robot_common_launch/common/launch_arg_utils.py) — profile、CLI、mappings（含 `create_platform_launch_arguments` / `resolve_robot_arms` / `planning_robot_for_arm_family`）
- [`robot_common_launch/common/launch_utils.py`](robot_common_launch/common/launch_utils.py) — 可视化 mappings / 通用 DeclareLaunchArgument
- [`robot_common_launch/common/robot_utils.py`](robot_common_launch/common/robot_utils.py) — `load_robot_config`（含 variant / hardware overlay）、planning URDF
- [`robot_common_launch/common/controller_manager_setup.py`](robot_common_launch/common/controller_manager_setup.py) — 向 `load_robot_config` 传入 `hardware`
