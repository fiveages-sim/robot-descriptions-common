# OyMotion DexHands Description

This package contains the URDF and related files for the OyMotion DexHands. Origin files could be found at [RoHand](https://github.com/oymotion/rohand_gen2_urdf_ros2).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to oymotion_description --symlink-install
```

## Visualize the DexHands

### RoHand Gen2 DexHands
* Left Hand
  ```bash
  # left hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=oymotion
  ```
  ![rohand gen2](../.images/rohand_gen2.png)
    
* Right Hand
  ```bash
  # right hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=oymotion direction:=-1
  ```