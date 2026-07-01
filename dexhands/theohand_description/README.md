# TheoHand Description

TheoHand STD16A 灵巧手的 URDF、ros2_control 接口和控制器配置。

当前支持两种真机接法：

- 单手 USB-RS485 调试：`basic_joint_controller hand.launch.py` + `modbus_ros2_control/TheoHandModbusHardware`
- Tianji / M6 CCS 臂端 485：`m6_ccs_description` + `marvin_ros2_control/MarvinHardware`

## 1. 文件结构

```text
theohand_description/
├── xacro/hand.xacro                    # 单手模型入口
├── xacro/std16a.xacro                  # STD16A 16 关节模型
├── xacro/ros2_control/hand.xacro       # 单手 ros2_control 入口
├── xacro/ros2_control/robot.xacro      # 兼容通用 controller_manager 的薄 wrapper
├── xacro/ros2_control/std16a.xacro     # 16 关节 ros2_control interface
├── config/ros2_control/std16a.yaml     # 单手控制器配置
└── meshes/std16a/                      # GLB mesh
```

`xacro/ros2_control/robot.xacro` 只有一行 include，用来兼容：

```bash
ros2 launch robot_common_launch controller_manager.launch.py robot:=theohand ...
```

正常单手调试走 `basic_joint_controller hand.launch.py`，它直接加载 `xacro/ros2_control/hand.xacro`。如果删掉 `robot.xacro`，通用 `controller_manager.launch.py robot:=theohand` 会再次报 `ros2_control xacro file not found`。

## 2. Build

```bash
cd /home/fa/fa_w2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to theohand_description --symlink-install
source install/setup.bash
```

真机还需要同时编译硬件包：

```bash
colcon build --packages-select modbus_ros2_control marvin_ros2_control theohand_description m6_ccs_description --symlink-install
source install/setup.bash
```

## 3. 只看模型

左手：

```bash
ros2 launch robot_common_launch hand.launch.py \
  hand:=theohand \
  type:=std16a \
  direction:=1
```

右手：

```bash
ros2 launch robot_common_launch hand.launch.py \
  hand:=theohand \
  type:=std16a \
  direction:=-1
```

`direction:=1` 是左手，`direction:=-1` 是右手。当前只实测右手，Tianji 臂端集成默认按右手地址 `1` 使用。

## 4. 单手 USB-RS485 调试

默认参数：

- 串口：`/dev/ttyUSB0`
- 波特率：`115200`
- Modbus 地址：`1`
- 反馈：开启
- 后台读反馈周期：`200 ms`
- 下发目标后反馈静默窗口：`500 ms`

启动右手：

```bash
ros2 launch basic_joint_controller hand.launch.py \
  hand:=theohand \
  type:=std16a \
  direction:=-1 \
  hardware:=real \
  hardware_serial_port:=/dev/ttyUSB0 \
  hardware_slave_id:=1 \
  hardware_baudrate:=115200
```

这些真机参数都有默认值，常规右手测试可以简化成：

```bash
ros2 launch basic_joint_controller hand.launch.py \
  hand:=theohand \
  type:=std16a \
  direction:=-1 \
  hardware:=real
```

如果想关掉反馈，只镜像命令到 RViz：

```bash
ros2 launch basic_joint_controller hand.launch.py \
  hand:=theohand \
  type:=std16a \
  direction:=-1 \
  hardware:=real \
  hardware_read_feedback:=false
```

## 5. 接 Tianji / M6 CCS 臂端

右手接臂端 485：

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=m6_ccs \
  hardware:=real \
  right_type:=theohand_std16a
```

不要传空值形式的 `left_type:=`，ROS launch 会报：

```text
malformed launch argument 'left_type:=', expected format '<name>:=<value>'
```

如果 profile 里默认带了左手末端，需要显式覆盖为空，可用：

```bash
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=m6_ccs \
  hardware:=real \
  left_type:=none \
  right_type:=theohand_std16a
```

Tianji 侧类型映射在：

- `m6_ccs_description/xacro/eef_type_resolver.xacro`
- `m6_ccs_description/xacro/eefs.xacro`
- `m6_ccs_description/xacro/ros2_control/end_effector_interfaces.xacro`

Marvin 侧创建工具类时识别：

- `theohand_std16a`
- `theohand`
- `std16a`

## 6. 控制与 Home 位

STD16A 导出 16 个 position command joint：

```text
thumb_joint1
thumb_joint2
thumb_joint3
thumb_joint4
index_joint1
index_joint2
index_joint3
middle_joint1
middle_joint2
middle_joint3
ring_joint1
ring_joint2
ring_joint3
pinky_joint1
pinky_joint2
pinky_joint3
```

控制器配置在 `config/ros2_control/std16a.yaml`：

- `home_1`：打开
- `home_2`：实测接近厂家软件全关，对应目标寄存器约 `9000`
- `home_3`：更大的中间测试姿态
- `target_command_close_config: 1`
- `target_command_open_config: 0`
- `home_interpolation_type: "none"`
- `movej_interpolation_type: "none"`

这里不做插值，RViz / FSM 发目标后直接下发整组关节目标。

## 7. 协议备注

STD16A 当前使用标准 Modbus RTU：

- 使能：FC06 写 `0x0000 = 0x000F`
- 目标位置：FC16 从 `0x0001` 写 16 个寄存器
- 实际位置：FC03 从 `0x0051` 读 16 个寄存器
- 位置量程：`0 ~ 9000`
- 反馈寄存器按 `int16_t` 解析，开位附近可能出现 `FFxx` 的负数噪声，需要按接近 0 处理

协议里的 `0x9007 LeftRight` 实测不可靠，当前不依赖它判断左右手。右手按 Modbus 地址 `1` 测试。

地址设置可按厂家协议：

- 写 `0x00B3 = 目标 Modbus 地址`
- 写 `0x00A3 = 12345` 保存

## 8. 常见问题

### 没有回包

先确认串口工具是 Hex 模式，不是 Plain 文本模式；CuteCom 需要把输入模式切到 `Hex`。基础读帧可用：

```text
01 03 00 51 00 10 15 DA
```

正常会返回 `01 03 20 ... CRC`。

### RViz 关上了，真手还是开着

通常是反馈解析或反馈关闭状态不一致导致：

- `hardware_read_feedback:=false` 时 RViz 显示命令态，不代表真实到位
- `hardware_read_feedback:=true` 时 RViz 显示真实反馈
- 如果开位读到 `FFxx`，必须按有符号值处理；当前硬件实现已经做了这个转换

### 下发后偶发超时 warning

`TheoHandModbusHardware` 会重试目标位置写入。若手动作和 RViz 同步，少量 warning 不一定影响使用；如果频繁出现，优先检查：

- USB-RS485 线和供电
- 串口是否被厂家软件或调试助手占用
- `background_period_ms` 是否过低
- 485 A/B 是否接反

## 9. 验证

渲染单手 URDF：

```bash
cd /home/fa/fa_w2_ws
source install/setup.bash
xacro "$(ros2 pkg prefix theohand_description)/share/theohand_description/xacro/hand.xacro" \
  type:=std16a direction:=-1 > /tmp/theohand_std16a_right.urdf
check_urdf /tmp/theohand_std16a_right.urdf
```

渲染接 M6 CCS 右手的 ros2_control URDF：

```bash
xacro "$(ros2 pkg prefix m6_ccs_description)/share/m6_ccs_description/xacro/ros2_control/robot.xacro" \
  ros2_control_hardware_type:=real \
  type:=dual \
  right_type:=theohand_std16a > /tmp/m6_theohand_right.urdf
```
