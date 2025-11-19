# Jodell Gripper Description

This package contains the URDF and related files for the Jodell RG75-300 Gripper.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to jodell_description --symlink-install
```

## Visualize the Gripper

* RG75-300 Gripper (无指套)
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell
  ```
  ![RG75](../.images/jodell_rg75.png)

* RG75-300 Gripper (带指套)
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=RG75-leapmotor
  ```

