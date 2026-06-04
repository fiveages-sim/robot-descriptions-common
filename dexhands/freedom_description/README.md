# Freedom DexHands Description

This package contains the URDF and related files for the Freedom dexterous hand.


## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to freedom_description --symlink-install
```

## 2. Visualize the DexHands

### Left Hand

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom
```

### Right Hand

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch hand.launch.py hand:=freedom direction:=-1
```

## 3. Model Notes

- `direction:=1` uses the left-hand model.
- `direction:=-1` mirrors the left-hand model to create the right hand.
- Link and joint names are normalized to match the existing dexhand packages.
- The source model contains 11 revolute joints. ROS2 control exposes 6 main
  control DOFs: 2 for the thumb and 1 for each of the other four fingers.

## 4. ROS2 Control

### 4.1 Controlled Joints

| Joint | DOF | Lower (rad) | Upper (rad) |
| --- | --- | ---: | ---: |
| `thumb_joint1` | Thumb 1 | 0.000 | 0.785 |
| `thumb_joint2` | Thumb 2 | 0.000 | 0.290 |
| `index_joint` | Index | 0.000 | 1.240 |
| `middle_joint` | Middle | 0.000 | 1.240 |
| `ring_joint` | Ring | 0.000 | 1.240 |
| `pinky_joint` | Pinky | 0.000 | 1.240 |

### 4.2 Visual/Kinematic Joints

These joints remain in the model for visualization and kinematics, but are not
commanded by `hand_joint_controller`:

| Joint | Mimic Source | Lower (rad) | Upper (rad) |
| --- | --- | ---: | ---: |
| `thumb_joint3` | `thumb_joint2` | 0.000 | 0.440 |
| `index_dip` | `index_joint` | 0.000 | 1.570 |
| `middle_dip` | `middle_joint` | 0.000 | 1.570 |
| `ring_dip` | `ring_joint` | 0.000 | 1.570 |
| `pinky_dip` | `pinky_joint` | 0.000 | 1.570 |

### 4.3 Launch

#### Left Hand

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom
```

#### Right Hand

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom direction:=-1
```

### 4.4 RS485 Hardware

Freedom uses a custom RS485 protocol instead of the LinkerHand Modbus protocol.
By default, `direction:=1` selects the left hand with ID `0`, and
`direction:=-1` selects the right hand with ID `1`. The default serial port is
`/dev/ttyACM0`.

Left hand (`direction:=1`, default):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real
```

Right hand (`direction:=-1`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real direction:=-1
```

### 4.5 CAN Hardware

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
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real_can
```

Right hand (`direction:=-1`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real_can direction:=-1
```
