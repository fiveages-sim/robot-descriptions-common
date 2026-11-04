# DH-Robotics Gripper and Dexterous Hand Descriptions

This package contains URDF/Xacro and meshes for DH-Robotics PGC grippers and the
DH-3-7 three-finger, seven-joint dexterous hand.

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
  ![pgc140](../.images/dh_pgc_140.png)

* PGC_140_50 Gripper (dexforce pad)
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch robot_common_launch gripper.launch.py gripper:=dh type:=PGC_140_50-dexforce
  ```

## 3. Visualize the DH-3-7

Build this package in the ROS 2 workspace, then source its environment:

```bash
colcon build --packages-select dh_description --symlink-install
source install/setup.bash
```

Right hand (`direction:=-1`):

```bash
ros2 launch robot_common_launch hand.launch.py hand:=dh type:=dh_3_7 direction:=-1
```

Left hand (`direction:=1`, the default):

```bash
ros2 launch robot_common_launch hand.launch.py hand:=dh type:=dh_3_7 direction:=1
```

The shared launch starts RViz and the joint-state publisher GUI. Use its seven
sliders to check the zero pose, finger axes, joint directions and limits. The
RViz fixed frame is `hand_base`. This visualization does not communicate with
the physical hand.

### Reusable model macro

```xml
<xacro:include filename="$(find dh_description)/xacro/dh_3_7.xacro"/>
<xacro:DH3_7 name="right_hand" direction="-1" fixed_joints="false"/>
```

`name` prefixes every link, joint and material; an empty name produces
`hand_base` and the unprefixed joint names below. A nonempty name produces, for
example, `right_hand_hand_base` and `right_hand_thumb_joint1`.
`fixed_joints="true"` replaces all seven revolute joints with fixed joints for
planning descriptions. Mounting to an arm requires a separately defined fixed
joint and a verified mounting transform; no flange or mount geometry is invented.

### Joint names and limits

Joints follow the existing `thumb_jointN`, `index_jointN`, `middle_jointN`
convention used by other dexterous hands (for example XHand1). The vendor
first, second and third finger chains are named thumb, index and middle.
Links follow `hand_<finger>_linkN`. With `name="right_hand"`, joints become
`right_hand_thumb_joint1`, etc.

| Vendor right / left joint | Xacro joint (before name prefix) | Right limits (rad) | Left limits (rad) |
|---|---|---|---|
| `F1R_Joint` / `LF1R_Joint` | `thumb_joint1` | 0 to 3.14 | -3.14 to 0 |
| `F11_Joint` / `LF11_Joint` | `thumb_joint2` | 0 to 3.14 | -3.14 to 0 |
| `F12_Joint` / `LF12_Joint` | `thumb_joint3` | 0 to 3.51 | -3.51 to 0 |
| `F21_Joint` / `LF21_Joint` | `index_joint1` | 0 to 3.14 | -3.14 to 0 |
| `F22_Joint` / `LF22_Joint` | `index_joint2` | 0 to 3.51 | -3.51 to 0 |
| `F31_Joint` / `LF31_Joint` | `middle_joint1` | 0 to 3.14 | -3.14 to 0 |
| `F32_Joint` / `LF32_Joint` | `middle_joint2` | 0 to 3.51 | -3.51 to 0 |

All joints use a velocity limit of **5 rad/s** and retain the vendor effort limit
of **10 N m**. The left `LF21_Joint` velocity was corrected from 51 to 5 at the
user's request. Origins, axes, masses, inertias, damping and friction otherwise
retain the vendor values. Physical joint directions and limits await visual
validation; these are the supplied model limits, not new measured limits.

### Asset provenance

- Left: user-provided `UDH3_7L.zip`, URDF
  `UDH3_7L_0903/urdf/UDH3_7L_0903.urdf`.
- Right: user-provided `UDH3_7R.zip`, URDF
  `UDH3_7R_0903/urdf/UDH3_7R_0903.urdf`.
- Left archive SHA-256:
  `3d97fb83ad216f2d9c18b90c7aaee0e51f4333df421de12644c5e1f040a16b7f`.
- Right archive SHA-256:
  `93fceaff51c00e1b0df721e44b2e616c30f1da4fb1157bdc99f9cb9948927dbd`.

Each side has eight original, unmodified STL files in
`meshes/dh_3_7/{left,right}/`. Visual and collision geometry use the original
meshes at scale 1. Left/right geometry and kinematics remain independent,
including the vendor's small wrist-origin differences. Original `base_link`
becomes `hand_base` without changing its coordinate frame. The models use the
original gray material; mesh conversion, repainting and simplified collision
geometry are deferred until visual validation.

### Confirmed device behavior for later driver integration

The following was verified with the connected right hand. No hardware/control
interface is added to this asset package yet.

- Position values 0 to 5000 represent normalized travel, with 0 at the end of
  homing and 5000 at maximum travel; do not interpret them as millimeters.
  The planned angle conversion divides each URDF joint range into 5000 parts;
  zero poses, axis mapping and directions still need model validation.
- After power-on, first read initialization status. If all seven 2-bit fields
  of `0x0201` equal 1 (`0x1555`), skip homing. Otherwise, when not already
  initializing, one `0x0100 = 0x2AAA` command was sufficient. Wait for all seven
  axes to report initialized and arrived; do not automatically append `0x15AA`
  or use a fixed delay as the completion condition.
- Homing can briefly report run state 2 or `0xFFFF`. Neither is completion;
  continue monitoring initialization, run state and fault codes.
- A single custom `0x6A` frame can set seven position, speed or target-force
  parameters. One `0x69` frame reads seven positions; 50 Hz polling was verified.
- Target speed is a percentage. Target force is a percentage in the documented
  range 20 to 100, not measured joint torque in N m. Runtime percentages do not
  replace the URDF velocity/effort limits.
