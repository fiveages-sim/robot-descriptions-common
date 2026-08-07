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
| `platform` | 始终 | 底盘 / `variant` / `chassis_joints_movable` 等 → xacro |
| `defaults.end_effectors` | 仅 `use_profile_eef:=true`（默认） | 末端类型；可被 CLI 关掉以改用 `type` / `left_type` / `right_type` |
| `defaults.ft` | 始终 | 力传感器；**不受** `use_profile_eef` 影响 |
| `defaults.tcp_offset` | 始终 | 虚拟 tip 偏移；注入前做表达式求值 |
| `hardware` | 仅 `hardware:=real` / `real_usb` | 真机参数（串口、`arm_ctrl_mode`、`*_dyn_param` 等） |
| `control.patch` | 始终 | 深合并进 ros2_control 配置 |

### Profile schema（包约定）

```yaml
platform:
  chassis: <key>
  variant: <key>
  chassis_joints_movable: "true"|"false"

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
| `chassis` / `variant` / `chassis_joints_movable` | 平台 xacro（人形 / 移动基座） |
| `collider` / `skin` / `direction` | 常用模型 xacro |

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

- [`robot_common_launch/common/launch_arg_utils.py`](robot_common_launch/common/launch_arg_utils.py) — profile、CLI、mappings
- [`robot_common_launch/common/launch_utils.py`](robot_common_launch/common/launch_utils.py) — 可视化 mappings / 通用 DeclareLaunchArgument
- [`robot_common_launch/common/robot_utils.py`](robot_common_launch/common/robot_utils.py) — ros2_control / planning URDF 生成
