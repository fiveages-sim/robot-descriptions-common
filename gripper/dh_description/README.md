# DH Gripper Description

This package contains the URDF and related files for the DH Grippers.

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to dh_description --symlink-install
```

## 2. Visualize the Gripper

* PGC_140_50 Gripper (default pad)
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=dh type:=PGC_140_50
  ```

* PGC_140_50 Gripper (dexforce pad)
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=dh type:=PGC_140_50-dexforce
  ```