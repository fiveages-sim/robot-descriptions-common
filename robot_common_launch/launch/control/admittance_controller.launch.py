"""
Admittance Controller Launch File

This launch file starts an admittance controller for force-controlled manipulation.
It uses the common controller_manager launch file for hardware configuration.

Usage:
    # With real hardware (force torque sensor FT-90C)
    ros2 launch robot_common_launch admittance_controller.launch.py robot:=cr5 type:=ft-90c hardware:=real

    # With mock components (for testing without hardware)
    ros2 launch robot_common_launch admittance_controller.launch.py robot:=cr5 hardware:=mock_components

    # With Gazebo simulation
    ros2 launch robot_common_launch admittance_controller.launch.py robot:=cr5 hardware:=gz type:=ft-90c world:=dart

    # Disable gripper controllers
    ros2 launch robot_common_launch admittance_controller.launch.py robot:=cr5 enable_gripper:=false

    # With RViz visualization
    ros2 launch robot_common_launch admittance_controller.launch.py robot:=cr5 use_rviz:=true
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
    enable_gripper = context.launch_configurations.get('enable_gripper', 'true').lower() == 'true'
    auto_enable_ft_sensor = context.launch_configurations.get('auto_enable_ft_sensor', 'true').lower() == 'true'
    force_service_prefix = context.launch_configurations.get('force_service_prefix', '/dobot_force_control/srv')
    force_control_mode = context.launch_configurations.get('force_control_mode', 'ros2_control_controllers')
    enable_gripper_io = context.launch_configurations.get('enable_gripper_io', 'true')

    # 根据 hardware 参数自动判断是否使用仿真时间
    use_sim_time = hardware in ['gz', 'isaac']

    # Load ros2_control config and robot_description once for validation.
    controller_config, controller_type_map = _get_controller_type_map(robot_name, robot_type)
    robot_description = get_ros2_control_robot_description(robot_name, robot_type, hardware)
    joint_command_interfaces, sensor_state_interfaces = _extract_ros2_control_interfaces(robot_description)

    # 使用通用的 controller manager launch 文件
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
            ('hardware', hardware),
            ('force_control_mode', force_control_mode),
            ('enable_gripper_io', enable_gripper_io),
        ],
    )

    # Admittance controller spawner
    admittance_controller_spawner = None
    if not _has_controller_type(controller_type_map, 'admittance_controller'):
        print("[WARN] 'admittance_controller' has no type in ros2_control config, skipping spawner")
    else:
        admittance_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['admittance_controller'],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )

    # Joint trajectory controller spawner (for position control when not in admittance mode)
    joint_trajectory_controller_spawner = None
    if not _has_controller_type(controller_type_map, 'joint_trajectory_controller'):
        print("[WARN] 'joint_trajectory_controller' has no type in ros2_control config, skipping spawner")
    else:
        joint_trajectory_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )

    # Detect hand controllers using robot_common_launch (only if gripper is enabled)
    hand_controllers = []
    hand_controller_spawners = []

    if enable_gripper:
        # Detect controllers matching hand/gripper patterns
        detected_controllers = detect_controllers(robot_name, robot_type, ['hand', 'gripper'], robot_description=robot_description)

        # Filter out invalid hand controllers
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

    # RViz node
    rviz_node = None
    if use_rviz:
        robot_common_launch_pkg = get_package_share_directory('robot_common_launch')
        rviz_config = os.path.join(
            robot_common_launch_pkg,
            'config',
            'rviz',
            'cartesian_controller.rviz'
        )
        if not os.path.exists(rviz_config):
            rviz_config = ""

        # Extract hand controller names for GripperControlPanel
        hand_controller_names = []
        if enable_gripper and hand_controllers:
            hand_controller_names = [c['name'] for c in hand_controllers]

        rviz_parameters = [{'use_sim_time': use_sim_time}]
        if hand_controller_names:
            rviz_parameters.append({'hand_controllers': hand_controller_names})

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config] if rviz_config else [],
            parameters=rviz_parameters,
        )

    # Collect all nodes
    nodes = [
        controller_manager_launch,
    ]

    # Add controller spawners if available
    if admittance_controller_spawner:
        nodes.append(admittance_controller_spawner)
    if joint_trajectory_controller_spawner:
        nodes.append(joint_trajectory_controller_spawner)

    # Add hand controller spawners if any were detected
    nodes.extend(hand_controller_spawners)

    if rviz_node:
        nodes.append(rviz_node)

    # For real hardware, enable FT sensor and zero at startup.
    if hardware == 'real' and auto_enable_ft_sensor:
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
        description='Robot type/variant (e.g., ft-90c)'
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

    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='true',
        description='Enable gripper controllers (automatically detects hand/gripper controllers)'
    )

    auto_enable_ft_sensor_arg = DeclareLaunchArgument(
        'auto_enable_ft_sensor',
        default_value='true',
        description='Auto call EnableFTSensor(1) and SixForceHome() for real hardware'
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
        enable_gripper_arg,
        auto_enable_ft_sensor_arg,
        force_service_prefix_arg,
        force_control_mode_arg,
        enable_gripper_io_arg,
        OpaqueFunction(function=launch_setup),
    ])
