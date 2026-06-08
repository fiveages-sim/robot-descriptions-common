# Freedom DexHands Description

This package contains the URDF and related files for the Freedom dexterous hand.

Supported model types:

- default / `freedomv2` / `freedom_v2`: Freedom V2 model migrated from `freedom_hand_description`.
- `freedomv1` / `freedom_v1` / `freedom`: original cleaned Freedom model.


## 1. Build

```bash
cd ~/fa_w2_ws
colcon build --packages-up-to freedom_description --symlink-install
```

## 2. Visualize the DexHands

Use `robot_common_launch` for RViz-only visualization with
`joint_state_publisher_gui`.

### Freedom V2 Left Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom direction:=1
```

### Freedom V2 Right Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom direction:=-1
```

### Freedom V1 Left Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom type:=freedomv1 direction:=1
```

### Freedom V1 Right Hand

```bash
source ~/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom type:=freedomv1 direction:=-1
```
cd /home/king/fa_w2_ws
source install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom direction:=1


## 3. Model Notes

- `direction:=1` selects the left hand.
- `direction:=-1` selects the right hand.
- V1 mirrors one GLB mesh set with `direction`.
- V2 reuses one GLB mesh set for both sides and mirrors the right
  hand with `direction`, while link and joint names are normalized by removing
  the original `l_` / `r_` prefixes.
- V1 contains 11 revolute joints. ROS2 control exposes 6 main control DOFs.
- V2 uses the common dexhand control naming: 3 thumb joints, 2 index joints,
  2 middle joints, 1 ring joint, and 1 pinky joint.

## 4. ROS2 Control

Use `basic_joint_controller` when you want RViz plus a ros2_control controller.
Always pass `type` with this launch file; its generic default is not a Freedom
model type.

### 4.1 Freedom V1 Controlled Joints

| Joint | DOF | Lower (rad) | Upper (rad) |
| --- | --- | ---: | ---: |
| `thumb_joint1` | Thumb 1 | 0.000 | 0.785 |
| `thumb_joint2` | Thumb 2 | 0.000 | 0.290 |
| `index_joint` | Index | 0.000 | 1.240 |
| `middle_joint` | Middle | 0.000 | 1.240 |
| `ring_joint` | Ring | 0.000 | 1.240 |
| `pinky_joint` | Pinky | 0.000 | 1.240 |

### 4.2 Freedom V1 Visual/Kinematic Joints

These joints remain in the model for visualization and kinematics, but are not
commanded by `hand_joint_controller`:

| Joint | Mimic Source | Lower (rad) | Upper (rad) |
| --- | --- | ---: | ---: |
| `thumb_joint3` | `thumb_joint2` | 0.000 | 0.440 |
| `index_dip` | `index_joint` | 0.000 | 1.570 |
| `middle_dip` | `middle_joint` | 0.000 | 1.570 |
| `ring_dip` | `ring_joint` | 0.000 | 1.570 |
| `pinky_dip` | `pinky_joint` | 0.000 | 1.570 |

### 4.3 Freedom V2 Controlled Joints

V2 exposes 9 semantic control joints:

```text
thumb_joint1
thumb_joint2
thumb_joint3
index_joint
index_dip
middle_joint
middle_dip
ring_joint
pinky_joint
```

With a side prefix in a robot, these become names such as
`left_hand_thumb_joint1` and `right_hand_thumb_joint1`.

### 4.4 Launch With Controller

#### Freedom V2 Left Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv2 direction:=1
```

#### Freedom V2 Right Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv2 direction:=-1
```

#### Freedom V1 Left Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 direction:=1
```

#### Freedom V1 Right Hand

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 direction:=-1
```

### 4.5 RS485 Hardware

Freedom uses a custom RS485 protocol instead of the LinkerHand Modbus protocol.
By default, `direction:=1` selects the left hand with ID `0`, and
`direction:=-1` selects the right hand with ID `1`. The default serial port is
`/dev/ttyACM0`.

The RS485 hardware driver supports both the V1 6-DOF protocol and the V2
9-DOF protocol. V2 uses the custom `0x06` motion command with angle, speed,
and current-limit bytes for each controlled joint.

Left hand (`direction:=1`, default):

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 hardware:=real direction:=1
```

Right hand (`direction:=-1`):

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 hardware:=real direction:=-1
```

Freedom V2 left hand:

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv2 hardware:=real direction:=1
```

Freedom V2 right hand:

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv2 hardware:=real direction:=-1
```

### 4.6 CAN Hardware

Freedom CAN uses CAN2.0 extended frames. Bring up the SocketCAN interface before
launching ROS2 control. The hardware supports `500K` and `1M`; use the bitrate
configured on the hand.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

Current Freedom IDs:

- `direction:=1`: left hand, default CAN device ID `0`
- `direction:=-1`: right hand, default CAN device ID `1`

Left hand (`direction:=1`, default):

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 hardware:=real_can direction:=1
```

Right hand (`direction:=-1`):

```bash
source ~/fa_w2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedomv1 hardware:=real_can direction:=-1
```
