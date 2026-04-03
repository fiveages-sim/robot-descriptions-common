"""
Cartesian Controller Launch File

This launch file starts a cartesian controller (motion, force, or compliance) for any robot.
It uses the common controller_manager launch file for hardware configuration.

Usage:
    # With real hardware (motion controller - default)
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=AG2F90-C-Soft

    # With force controller
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=AG2F90-C-Soft controller_type:=force

    # With compliance controller
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=real type:=AG2F90-C-Soft controller_type:=compliance

    # With Gazebo simulation
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=gz type:=AG2F90-C-Soft world:=dart

    # With mock components (for testing)
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 hardware:=mock_components

    # With interactive marker handle (RViz visualization)
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 use_rviz:=true use_handle:=true

    # Disable gripper controllers
    ros2 launch robot_common_launch cartesian_controller.launch.py robot:=cr5 enable_gripper:=false
"""

import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Import robot_common_launch utilities for automatic hand controller detection
from robot_common_launch import (
    detect_controllers,
    create_controller_spawners,
    get_ros2_control_robot_description,
    load_robot_config,
)


def _extract_ros2_control_interfaces(robot_description):
    """
    Parse ros2_control interfaces from robot_description.

    Returns:
        tuple(dict, dict): (joint_command_interfaces, sensor_state_interfaces)
    """
    joint_command_interfaces = {}
    sensor_state_interfaces = {}

    if not robot_description:
        return joint_command_interfaces, sensor_state_interfaces

    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as e:
        print(f"[WARN] Failed to parse robot_description for interface validation: {e}")
        return joint_command_interfaces, sensor_state_interfaces

    for ros2_control in root.findall('.//ros2_control'):
        for joint in ros2_control.findall('joint'):
            joint_name = joint.get('name')
            if not joint_name:
                continue
            interfaces = joint_command_interfaces.setdefault(joint_name, set())
            for cmd_if in joint.findall('command_interface'):
                interface_name = cmd_if.get('name')
                if interface_name:
                    interfaces.add(interface_name)

        for sensor in ros2_control.findall('sensor'):
            sensor_name = sensor.get('name')
            if not sensor_name:
                continue
            interfaces = sensor_state_interfaces.setdefault(sensor_name, set())
            for state_if in sensor.findall('state_interface'):
                interface_name = state_if.get('name')
                if interface_name:
                    interfaces.add(interface_name)

    return joint_command_interfaces, sensor_state_interfaces


def _get_controller_type_map(robot_name, robot_type):
    config, _ = load_robot_config(robot_name, "ros2_control", robot_type)
    if config is None:
        return {}, {}

    controller_manager = config.get('controller_manager', {}).get('ros__parameters', {})
    controller_type_map = {}
    for name, cfg in controller_manager.items():
        if isinstance(cfg, dict):
            controller_type_map[name] = cfg.get('type', '')

    return config, controller_type_map


def _has_controller_type(controller_type_map, controller_name):
    return bool(controller_type_map.get(controller_name, ''))


def launch_setup(context, *args, **kwargs):
    """Launch setup function using OpaqueFunction"""
    robot_name = context.launch_configurations.get('robot', 'cr5')
    robot_type = context.launch_configurations.get('type', '')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    world = context.launch_configurations.get('world', 'dart')
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    use_handle = context.launch_configurations.get('use_handle', 'true').lower() == 'true'
    enable_gripper = context.launch_configurations.get('enable_gripper', 'true').lower() == 'true'
    controller_type = context.launch_configurations.get('controller_type', 'motion').lower()
    auto_enable_ft_sensor = context.launch_configurations.get('auto_enable_ft_sensor', 'true').lower() == 'true'
    force_service_prefix = context.launch_configurations.get('force_service_prefix', '/dobot_force_control/srv')
    force_control_mode = context.launch_configurations.get('force_control_mode', 'ros2_control_controllers')
    enable_gripper_io = context.launch_configurations.get('enable_gripper_io', 'true')

    # 验证控制器类型
    valid_controller_types = ['motion', 'force', 'compliance']
    if controller_type not in valid_controller_types:
        raise ValueError(f"Invalid controller_type: {controller_type}. Must be one of {valid_controller_types}")

    # 根据 hardware 参数自动判断是否使用仿真时间
    use_sim_time = hardware in ['gz', 'isaac']

    # Load ros2_control config and robot_description once for validation.
    controller_config, controller_type_map = _get_controller_type_map(robot_name, robot_type)
    robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
    joint_command_interfaces, sensor_state_interfaces = _extract_ros2_control_interfaces(robot_description)

    # 构建 remappings 字符串（格式: "from1:to1;from2:to2"）
    # 将控制器话题映射到统一的话题（不带前导斜杠，与 cartesian_controller_simulation 一致）
    remappings_list = []
    if use_handle:
        remappings_list.append('motion_control_handle/target_frame:target_frame')
    
    # 根据控制器类型添加相应的 remappings
    if controller_type == 'motion':
        remappings_list.append('cartesian_motion_controller/target_frame:target_frame')
    elif controller_type == 'force':
        remappings_list.append('cartesian_force_controller/target_wrench:target_wrench')
        remappings_list.append('cartesian_force_controller/ft_sensor_wrench:force_torque_sensor_broadcaster/wrench_filtered')
    elif controller_type == 'compliance':
        remappings_list.append('cartesian_compliance_controller/target_frame:target_frame')
        remappings_list.append('cartesian_compliance_controller/target_wrench:target_wrench')
        remappings_list.append('cartesian_compliance_controller/ft_sensor_wrench:force_torque_sensor_broadcaster/wrench_filtered')
        remappings_list.append('cartesian_compliance_controller/current_pose:left_current_pose')
    
    # 同时映射其他控制器订阅的话题到通用话题（与simulation.launch.py保持一致）
    # 这些 remappings 用于确保所有控制器都能访问通用话题
    remappings_str = ';'.join(remappings_list)

    # 使用通用的 controller manager launch 文件
    # 它处理了机器人描述生成、robot_state_publisher、Gazebo 启动等
    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_common_launch'), 'launch'),
            '/controller_manager.launch.py',
        ]),
        launch_arguments=[
            ('robot', robot_name),
            ('type', robot_type),
            ('use_sim_time', str(use_sim_time)),
            ('world', world),
            ('hardware', hardware),  # 传递硬件类型，controller_manager 会根据此参数自动判断是否使用 Gazebo
            ('force_control_mode', force_control_mode),
            ('enable_gripper_io', enable_gripper_io),
            ('remappings', remappings_str),  # 传递 remappings 参数
        ],
    )

    # Cartesian controller spawner (根据 controller_type 选择)
    cartesian_controller_spawner = None
    if controller_type == 'motion':
        if not _has_controller_type(controller_type_map, 'cartesian_motion_controller'):
            print("[WARN] 'cartesian_motion_controller' has no type in ros2_control config, skipping spawner")
        else:
            cartesian_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['cartesian_motion_controller'],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/cartesian_motion_controller/target_frame', '/target_frame')
                ]
            )
    elif controller_type == 'force':
        if not _has_controller_type(controller_type_map, 'cartesian_force_controller'):
            print("[WARN] 'cartesian_force_controller' has no type in ros2_control config, skipping spawner")
        else:
            cartesian_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['cartesian_force_controller'],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/cartesian_force_controller/target_wrench', '/target_wrench'),
                    ('/cartesian_force_controller/ft_sensor_wrench', '/ft_sensor_wrench')
                ]
            )
    elif controller_type == 'compliance':
        if not _has_controller_type(controller_type_map, 'cartesian_compliance_controller'):
            print("[WARN] 'cartesian_compliance_controller' has no type in ros2_control config, skipping spawner")
        else:
            cartesian_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['cartesian_compliance_controller'],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/cartesian_compliance_controller/target_frame', '/target_frame'),
                    ('/cartesian_compliance_controller/target_wrench', '/target_wrench'),
                    ('/cartesian_compliance_controller/ft_sensor_wrench', '/ft_sensor_wrench')
                ]
            )

    # Force torque sensor broadcaster spawner
    force_torque_sensor_broadcaster_spawner = None
    if controller_type in ['force', 'compliance']:
        required_ft_interfaces = {'force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z'}
        available_ft_interfaces = sensor_state_interfaces.get('ft_sensor', set())

        if not _has_controller_type(controller_type_map, 'force_torque_sensor_broadcaster'):
            print("[WARN] 'force_torque_sensor_broadcaster' has no type in ros2_control config, skipping spawner")
        elif not required_ft_interfaces.issubset(available_ft_interfaces):
            print(
                "[WARN] Skip 'force_torque_sensor_broadcaster': missing ft_sensor state interfaces in ros2_control "
                f"(required={sorted(required_ft_interfaces)}, available={sorted(available_ft_interfaces)})"
            )
        else:
            force_torque_sensor_broadcaster_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['force_torque_sensor_broadcaster'],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
            )

    # Motion control handle spawner
    # Remap target_frame to unified /target_frame topic
    motion_control_handle_spawner = None
    if use_handle:
        if not _has_controller_type(controller_type_map, 'motion_control_handle'):
            print("[WARN] 'motion_control_handle' has no type in ros2_control config, skipping spawner")
        else:
            motion_control_handle_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=['motion_control_handle'],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('/motion_control_handle/target_frame', '/target_frame')
                ]
            )

    # Detect hand controllers using robot_common_launch (only if gripper is enabled)
    hand_controllers = []
    hand_controller_spawners = []
    
    if enable_gripper:
        # Detect controllers matching hand/gripper patterns
        # Pass robot_description to verify joints exist in xacro
        detected_controllers = detect_controllers(robot_name, robot_type, ['hand', 'gripper'], robot_description=robot_description)
        
        # Filter out invalid hand controllers:
        # 1) motion_control_handle is not a gripper controller
        # 2) controller must have a declared type
        # 3) controller's configured joint must expose position command interface in ros2_control
        for controller in detected_controllers:
            controller_name = controller['name']
            if controller_name == 'motion_control_handle':
                continue

            if not _has_controller_type(controller_type_map, controller_name):
                print(f"[WARN] Skip hand controller '{controller_name}': missing type in controller_manager config")
                continue

            controller_params = controller_config.get(controller_name, {}).get('ros__parameters', {}) if controller_config else {}
            joint_name = controller_params.get('joint')
            if joint_name:
                available_joint_interfaces = joint_command_interfaces.get(joint_name, set())
                if 'position' not in available_joint_interfaces:
                    print(
                        f"[WARN] Skip hand controller '{controller_name}': joint '{joint_name}' has no "
                        f"'position' command interface in ros2_control (available={sorted(available_joint_interfaces)})"
                    )
                    continue

            hand_controllers.append(controller)
        
        hand_controller_spawners = create_controller_spawners(hand_controllers, use_sim_time)

        # Fallback for CR5-like single gripper setups:
        # If interface validation filtered out everything, but config still defines
        # a standard hand_controller, force-enable it so RViz gripper panel can work.
        if not hand_controllers and _has_controller_type(controller_type_map, 'hand_controller'):
            print("[WARN] No hand controller detected after validation, fallback to 'hand_controller'")
            hand_controllers = [{
                'name': 'hand_controller',
                'type': controller_type_map.get('hand_controller', ''),
                'config': {}
            }]
            hand_controller_spawners = create_controller_spawners(hand_controllers, use_sim_time)

    # RViz node
    rviz_node = None
    if use_rviz:
        robot_common_launch_pkg = get_package_share_directory('robot_common_launch')
        # Use cartesian_controller.rviz as default RViz config for cartesian controllers
        rviz_config = os.path.join(
            robot_common_launch_pkg,
            'config',
            'rviz',
            'cartesian_controller.rviz'
        )
        # Fallback to default RViz if config doesn't exist
        if not os.path.exists(rviz_config):
            rviz_config = ""
        
        # Extract hand controller names for GripperControlPanel
        hand_controller_names = []
        if enable_gripper and hand_controllers:
            hand_controller_names = [c['name'] for c in hand_controllers]
        
        # Prepare RViz parameters
        rviz_parameters = [{'use_sim_time': use_sim_time}]
        
        # Only add hand_controllers parameter if we have controllers
        if hand_controller_names:
            rviz_parameters.append({'hand_controllers': hand_controller_names})

        print(f"rviz_parameters: {rviz_parameters}")
        
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config] if rviz_config else [],
            parameters=rviz_parameters,
        )

    # Collect all nodes
    # controller_manager_launch 已经包含了：
    # - robot_state_publisher
    # - Gazebo 相关节点（如果 hardware=gz）
    # - ros2_control_node（如果 hardware!=gz）
    # - joint_state_broadcaster spawner
    nodes = [
        controller_manager_launch,
        cartesian_controller_spawner,
    ]

    if force_torque_sensor_broadcaster_spawner:
        nodes.append(force_torque_sensor_broadcaster_spawner)
    
    # 添加 motion_control_handle spawner（如果启用）
    if motion_control_handle_spawner:
        nodes.append(motion_control_handle_spawner)
    
    # Add hand controller spawners if any were detected
    nodes.extend(hand_controller_spawners)
    
    if rviz_node:
        nodes.append(rviz_node)

    # For real hardware force/compliance modes, enable FT sensor and zero at startup.
    # In ros2_control_controllers mode, only calibration services are exposed
    # (EnableFTSensor/SixForceHome/GetForce), while native drag services stay disabled.
    if (hardware == 'real' and controller_type in ['force', 'compliance'] and
            auto_enable_ft_sensor):
        enable_ft_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                f'{force_service_prefix}/EnableFTSensor',
                'dobot_force_msgs/srv/EnableFTSensor',
                '{status: 1}'
            ],
            output='screen',
        )
        nodes.append(TimerAction(period=2.0, actions=[enable_ft_cmd]))
        six_force_home_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                f'{force_service_prefix}/SixForceHome',
                'dobot_force_msgs/srv/SixForceHome',
                '{}'
            ],
            output='screen',
        )
        nodes.append(TimerAction(period=3.5, actions=[six_force_home_cmd]))

    return nodes


def generate_launch_description():
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='cr5',
        description='Robot name (default: cr5)'
    )

    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='mock_components',
        description='Hardware type: real, gz (Gazebo), mock_components, or isaac'
    )

    type_arg = DeclareLaunchArgument(
        'type',
        default_value='',
        description='Robot type/variant (e.g., AG2F90-C-Soft)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='dart',
        description='Gazebo world file name (only used when hardware=gz)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz visualization'
    )

    use_handle_arg = DeclareLaunchArgument(
        'use_handle',
        default_value='true',
        description='Whether to launch motion control handle (interactive marker)'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='motion',
        description='Controller type: motion, force, or compliance (default: motion)'
    )

    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='true',
        description='Enable gripper controllers (automatically detects hand/gripper controllers)'
    )

    auto_enable_ft_sensor_arg = DeclareLaunchArgument(
        'auto_enable_ft_sensor',
        default_value='true',
        description='Auto call EnableFTSensor(1) and SixForceHome() for real hardware force/compliance modes'
    )

    force_service_prefix_arg = DeclareLaunchArgument(
        'force_service_prefix',
        default_value='/dobot_force_control/srv',
        description='Force service prefix, e.g. /dobot_force_control/srv'
    )

    force_control_mode_arg = DeclareLaunchArgument(
        'force_control_mode',
        default_value='ros2_control_controllers',
        description='Dobot force mode: native_commands or ros2_control_controllers'
    )

    enable_gripper_io_arg = DeclareLaunchArgument(
        'enable_gripper_io',
        default_value='true',
        description='Whether to enable gripper Modbus I/O in dobot hardware'
    )

    return LaunchDescription([
        robot_arg,
        hardware_arg,
        type_arg,
        world_arg,
        use_rviz_arg,
        use_handle_arg,
        controller_type_arg,
        enable_gripper_arg,
        auto_enable_ft_sensor_arg,
        force_service_prefix_arg,
        force_control_mode_arg,
        enable_gripper_io_arg,
        OpaqueFunction(function=launch_setup),
    ])
