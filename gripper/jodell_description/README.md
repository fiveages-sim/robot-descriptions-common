# Jodell Gripper Description

This package contains the URDF and related files for Jodell grippers (RG75-300, ERG32).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to jodell_description --symlink-install
```

## Visualize the Gripper

### RG75-300 Series

* No Pad
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell
  ```
* FiveAges Pad
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell
  ```
  ![RG75](../.images/jodell_rg75.png)

* leapmotor pad
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=RG75-leapmotor
  ```

* Heavy carry pad
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=heavy_carry_left
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=heavy_carry_right
  ```

* Ptc pad
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=RG75-ptc
  ```

### ERG32 Rotating Gripper

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_common_launch gripper.launch.py gripper:=jodell type:=ERG32
```

![ERG32](../.images/jodell_erg32.png)