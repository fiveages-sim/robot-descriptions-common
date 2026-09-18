# BrainCo DexHands Description

This package contains the URDF and related files for the Brainco DexHands. Origin files could be found at [BrainCo](https://www.brainco.cn/#/product/revo2).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to brainco_description --symlink-install
```

## Visualize the DexHands

### Revo2 DexHands
* Left Hand
  ```bash
  # left hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=brainco direction:=1
  ```
  ![revo2 left](../.images/brainco_revo2.png)
    
* Right Hand
  ```bash
  # right hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=brainco direction:=-1
  ```

### Revo1 DexHands
![revo2 left](../.images/brainco_revo1.png)
* Left Hand
  ```bash
  # left hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=brainco type:=Revo1
  ```

* Right Hand
  ```bash
  # right hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=brainco type:=Revo1 direction:=-1
  ```

## ROS2 Control

Hand controllers live in this package (`config/ros2_control/`). Robots with `type:=revo1` / `revo2` compose `{side}_hand_controller` from `templates/revo1.side.yaml` / `revo2.side.yaml` via `eef_control_registry.yaml`.

Standalone mock:

```bash
ros2 launch basic_joint_controller hand.launch.py hand:=brainco type:=revo2
ros2 launch basic_joint_controller hand.launch.py hand:=brainco type:=revo1 direction:=-1
```

| Home | Pose |
|------|------|
| `home_1` | fully open |
| `home_2` | thumb opposed, fingers open |
| `home_3` | thumb + fingers closed |

`target_command` 0 = close (`home_3`), 1 = open (`home_2`).
