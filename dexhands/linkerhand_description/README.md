# LinkerHand DexHands Description

This package contains the URDF and related files for the LinkerHand DexHands. Origin files could be found at [LinkerHand](https://github.com/linker-bot/linkerhand-urdf).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to linkerhand_description --symlink-install
```

## Visualize the DexHands

### O7 DexHands
* Left Hand
  ```bash
  # left hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=linkerhand
  ```
  ![linkerhand o7](../.images/linkerhand_o7.png)
    
* Right Hand
  ```bash
  # right hand
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch hand.launch.py hand:=linkerhand direction:=-1
  ```