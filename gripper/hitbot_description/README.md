# Hitbot Description

This package contains the URDF and related files for Hitbot grippers.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to hitbot_description --symlink-install
```

## Visualize the Gripper

* Z-EFG-100
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=hitbot
  ```

  ![Z-EFG-100](../.images/hitbot_z_efg_100.png)

## ROS2 Control Demo

* Z-EFG-100
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch adaptive_gripper_controller demo.launch.py gripper:=hitbot
```
