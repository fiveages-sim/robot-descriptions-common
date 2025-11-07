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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Import robot_common_launch utilities for automatic hand controller detection
from robot_common_launch import (
    detect_controllers,
    create_controller_spawners
)


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

    # 验证控制器类型
    valid_controller_types = ['motion', 'force', 'compliance']
    if controller_type not in valid_controller_types:
        raise ValueError(f"Invalid controller_type: {controller_type}. Must be one of {valid_controller_types}")

    # 根据 hardware 参数自动判断是否使用仿真时间
    use_sim_time = hardware in ['gz', 'isaac']

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
    elif controller_type == 'compliance':
        remappings_list.append('cartesian_compliance_controller/target_frame:target_frame')
        remappings_list.append('cartesian_compliance_controller/target_wrench:target_wrench')
    
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
            ('remappings', remappings_str),  # 传递 remappings 参数
        ],
    )

    # Cartesian controller spawner (根据 controller_type 选择)
    cartesian_controller_spawner = None
    if controller_type == 'motion':
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

    # Motion control handle spawner
    # Remap target_frame to unified /target_frame topic
    motion_control_handle_spawner = None
    if use_handle:
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
        detected_controllers = detect_controllers(robot_name, robot_type, ['hand', 'gripper'])
        
        # Filter out motion_control_handle (it's not a gripper controller)
        # motion_control_handle is an interactive marker handle for RViz drag-and-drop control
        hand_controllers = [
            c for c in detected_controllers 
            if c['name'] != 'motion_control_handle'
        ]
        
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
    
    # 添加 motion_control_handle spawner（如果启用）
    if motion_control_handle_spawner:
        nodes.append(motion_control_handle_spawner)
    
    # Add hand controller spawners if any were detected
    nodes.extend(hand_controller_spawners)
    
    if rviz_node:
        nodes.append(rviz_node)

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

    return LaunchDescription([
        robot_arg,
        hardware_arg,
        type_arg,
        world_arg,
        use_rviz_arg,
        use_handle_arg,
        controller_type_arg,
        enable_gripper_arg,
        OpaqueFunction(function=launch_setup),
    ])

