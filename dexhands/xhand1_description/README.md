# XHAND1 DexHands Description

This package contains the URDF and related files for the XHAND1 DexHands.
Origin files could be found at `src/XHAND1_URDF_ver 1.3`.

## 1. Build

```bash
cd ~/fa_w2_ws
colcon build --packages-up-to xhand1_description --symlink-install
```

## 2. Visualize the DexHands

### 2.1 XHAND1 DexHands
* Left Hand
  ```bash
  # left hand
  source ~/fa_w2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=xhand1
  ```

* Right Hand
  ```bash
  # right hand
  source ~/fa_w2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=xhand1 direction:=-1
  ```

## 3. ROS2 Control Demo

### 3.1 XHAND1 DexHands
* Left Hand
  ```bash
  source ~/fa_w2_ws/install/setup.bash
  ros2 launch basic_joint_controller hand.launch.py hand:=xhand1 type:=xhand1 direction:=1
  ```

* Right Hand
  ```bash
  source ~/fa_w2_ws/install/setup.bash
  ros2 launch basic_joint_controller hand.launch.py hand:=xhand1 type:=xhand1 direction:=-1
  ```
