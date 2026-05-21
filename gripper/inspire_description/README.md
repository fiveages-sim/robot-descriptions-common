# Inspire Description

This package contains the URDF and related files for Inspire grippers and dexterous hands.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to inspire_description --symlink-install
```

## Visualize the Gripper

* EG2-4C2
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=inspire
  ```

  ![EG2](../.images/inspire_eg2.png)

## Visualize the Dexterous Hand

### RH56F2 Hand
Direction convention: `1` = left hand, `-1` = right hand.
* Left
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=inspire type:=RH56F2
  ```
  ![RH56F2](../../dexhands/.images/inspire_rh56f2.png)
* Right
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=inspire type:=RH56F2 direction:=-1
  ```

### RH56E2 Hand
Direction convention: `1` = left hand, `-1` = right hand.
* Left
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=inspire type:=RH56E2
  ```
  ![RH56E2](../../dexhands/.images/inspire_rh56e2.png)
* Right
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=inspire type:=RH56E2 direction:=-1
  ```
  ![RH56E2_Right](../../dexhands/.images/inspire_rh56e2_right.png)

## ROS2 Control Demo (Dexterous Hand)

### RH56F2 Hand
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=inspire type:=RH56F2
```

### RH56E2 Hand
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=inspire type:=RH56E2
```