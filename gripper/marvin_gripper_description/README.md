# Marvin Gripper Description

Tianji Marvin gripper descriptions extracted from the Marvin Pro robot package:

- `marvin_gripper` — straight flange variant
- `marvin_gripper45` — 45-degree flange variant

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to marvin_gripper_description --symlink-install
```

## Visualize

### Straight flange (default)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch gripper.launch.py gripper:=marvin_gripper
```

![marvin_gripper](../.images/marvin_gripper.png)

### 45-degree flange

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch gripper.launch.py gripper:=marvin_gripper type:=marvin_gripper45
```

![marvin_gripper45](../.images/marvin_gripper_45.png)

## Usage in robot xacro

```xml
<xacro:include filename="$(find marvin_gripper_description)/xacro/marvin_gripper.xacro"/>
<xacro:marvin_gripper name="left"/>

<!-- or -->
<xacro:include filename="$(find marvin_gripper_description)/xacro/marvin_gripper45.xacro"/>
<xacro:marvin_gripper45 name="right"/>

<!-- gripper core only (no flange / camera) -->
<xacro:include filename="$(find marvin_gripper_description)/xacro/components/marvin_gripper.xacro"/>
<xacro:marvin_gripper_component name="left"/>

<!-- straight flange + camera only (expects gripper_base) -->
<xacro:include filename="$(find marvin_gripper_description)/xacro/components/flange.xacro"/>
<xacro:marvin_gripper_flange name="left"/>

<!-- 45-degree flange + camera only (expects gripper_base) -->
<xacro:include filename="$(find marvin_gripper_description)/xacro/components/flange45.xacro"/>
<xacro:marvin_gripper_flange45 name="left"/>
```

Attach the arm to `${prefix}flange`. The TCP frame is `${prefix}eef`.

The gripper model is already rotated **-90° about Z** relative to the raw Marvin Pro URDF meshes
(same as the original `flange -> gripper_mount` yaw), so arm attach can use identity:

```xml
<joint name="arm_to_gripper" type="fixed">
  <parent link="arm_flange"/>
  <child link="flange"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```
