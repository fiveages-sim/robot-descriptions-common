# Wuji DexHands Description

URDF / xacro for Wuji dexterous hands, adapted for motion control in `robot-descriptions-common/dexhands` (fa_w2 / ROS 2 Jazzy).

Official source assets (geometry reference only): [wuji-description](https://github.com/wuji-technology/wuji-description)

**Included types**

| `type` | Official source | Notes |
|--------|-----------------|-------|
| `hand2` (default) | `hand2/hand2_beta2` | 20-DOF anatomical joint names + tip sensor frames |
| `hand1` | `hand/body` | 20-DOF `finger*_joint*` names |

Do **not** use `type:=hand` (legacy); it will not load the gen1 model or `hand1.yaml`.

Not packaged: `hand_soft`, `hand2_beta1`, `glove`, MJCF/USD/STEP.

Naming: `hand:=wuji` selects this package; `type` selects the hand model. Entry files are still named `hand.xacro`.

## 1. Build

```bash
cd ~/ros2_ws   # or this workspace
colcon build --packages-select wuji_description --symlink-install
source install/setup.bash
```

## 2. Visualize the DexHands

### 2.1 Hand2 (Beta 2, default)

```bash
ros2 launch robot_common_launch hand.launch.py hand:=wuji
# right:
ros2 launch robot_common_launch hand.launch.py hand:=wuji direction:=-1
# optional: use_mount:=true
```

### 2.2 Hand1 (gen1)

```bash
ros2 launch robot_common_launch hand.launch.py hand:=wuji type:=hand1
# right:
ros2 launch robot_common_launch hand.launch.py hand:=wuji type:=hand1 direction:=-1
```

## 3. ROS2 Control Demo (mock)

Uses `basic_joint_controller` with `mock_components` by default. Controllers load `config/ros2_control/{type}.yaml`.

```bash
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand2 direction:=-1

ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand1
ros2 launch basic_joint_controller hand.launch.py hand:=wuji type:=hand1 direction:=-1
```

Confirm the log shows `hand2.yaml` / `hand1.yaml` (not a failed progressive match falling back unexpectedly for the wrong type).

Open/close (when controller is active):

```bash
ros2 topic pub --once /hand_joint_controller/target_command std_msgs/msg/Int32 "data: 0"  # close -> home_2
ros2 topic pub --once /hand_joint_controller/target_command std_msgs/msg/Int32 "data: 1"  # open  -> home_1
```

## 4. Joint names

Standalone uses empty `name` prefix. On an arm, pass `name:=left_hand` / `right_hand` → `left_hand_<joint>`.

### 4.1 Hand2 (`type:=hand2`)

| Finger | Joints |
|--------|--------|
| Thumb | `thumb_cmc_flex`, `thumb_cmc_abd`, `thumb_mcp`, `thumb_ip` |
| Index | `index_finger_mcp_flex`, `index_finger_mcp_abd`, `index_finger_pip`, `index_finger_dip` |
| Middle | `middle_finger_mcp_flex`, `middle_finger_mcp_abd`, `middle_finger_pip`, `middle_finger_dip` |
| Ring | `ring_mcp_flex`, `ring_mcp_abd`, `ring_pip`, `ring_dip` |
| Pinky | `pinky_mcp_flex`, `pinky_mcp_abd`, `pinky_pip`, `pinky_dip` |

Ring names drop the official `finger` token (`ring_finger_*` → `ring_*`) so standalone RViz JointControlPanel does not mis-classify them as `right_hand` (leading `r` + `finger` heuristic). Links follow the same rename (`ring_proximal`, …). SDK / firmware order is unchanged — map by index in a future HI.

Fixed tip / tip-sensor frames exist for TF but are not commanded.

### 4.2 Hand1 (`type:=hand1`)

| Finger | Joints |
|--------|--------|
| Thumb (finger1) | `finger1_joint1` … `finger1_joint4` |
| Index (finger2) | `finger2_joint1` … `finger2_joint4` |
| Middle (finger3) | `finger3_joint1` … `finger3_joint4` |
| Ring (finger4) | `finger4_joint1` … `finger4_joint4` |
| Pinky (finger5) | `finger5_joint1` … `finger5_joint4` |

## 5. L/R mirroring and frames

### Hand2 wrist frames

| Link | Role |
|------|------|
| `hand_base` | External wrist for fa_w2 / dexhands; **+Z = finger extension** (same idea as LinkerHand O6) |
| `wrist` | Official beta2 wrist mesh + finger roots; fingers extend in **−Z** of this frame |
| `wrist_align` | Fixed `R_x(π)` mapping official −Z onto `hand_base` +Z |

Joint names, order, and limits are unchanged — simulation controllers and a future HardwareInterface still use joint-space angles (no sign flip for +Z). When mounting on an arm, calibrate `*_hand_base_joint` against this **+Z** `hand_base`.

- `direction:=1` left (default), `direction:=-1` right
- Single geometric tree + `direction` formulas (joint origins / axes)
- **Hand1 mesh:** `meshes/hand1/{left,right}/` side STL, **no** Y-scale. Four-finger segments share `digit_*` meshes.
- **Hand2 mesh:** left STL + `scale="1 ${direction} 1"`. Four-finger segments share `digit_*` (pinky `proximal_abd` / `middle` stay unique).
- **Hand1** `finger1_joint2` / `finger1_joint3`: **rpy ternary** (official L/R); do not hard-mirror those rpy fields
- **Hand2** `thumb_mcp` / `pinky_mcp_flex`: forced mirror for joint-space control (~1–3 mm fingertip FK vs official right URDF)

Whole-robot attach pattern (same as LinkerHand):

```xml
<xacro:WujiHand2 name="${side}_hand" direction="${1 if side == 'left' else -1}" use_mount="true"/>
```

Side controller templates: `config/ros2_control/templates/hand1.side.yaml`, `hand2.side.yaml`.

## 6. Real hardware (fa_w2)

This package does **not** integrate the official [wujihandros2](https://github.com/wuji-technology/wujihandros2) topic driver.

- Current demos: `mock_components` / `gz` / `isaac`
- `ros2_control_hardware_type:=real` selects placeholder plugin `wuji_ros2_control/WujiHandHardware` (`hand_type` = `hand1`|`hand2`, `hand_side` from `direction`) — **not implemented**; reserved for a future fa_w2 HardwareInterface (hand2 first)

Controller YAML contract for that HI: 20 joints in `hand2.yaml` / `hand1.yaml`; at least `position` command/state (xacro also declares velocity/effort).

Agent notes for HI + shared-stack policy: workspace `.cursor/rules/wuji-hand2-hardware.mdc` and `shared-control-stack.mdc`.

## 7. Package layout

```
wuji_description/
├── meshes/hand1/{left,right}/   # gen1 side STLs + digit_* share
├── meshes/hand2/left/           # beta2 left + digit_* (+ Y-scale in xacro)
├── xacro/hand.xacro             # type + direction dispatcher
├── xacro/hand1.xacro            # WujiHand
├── xacro/hand2.xacro            # WujiHand2
├── xacro/ros2_control/
└── config/ros2_control/         # hand1.yaml, hand2.yaml, templates/
```
