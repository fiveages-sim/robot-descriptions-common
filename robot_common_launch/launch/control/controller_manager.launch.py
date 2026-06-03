"""
通用的 Controller Manager launch 文件

这个文件负责启动 controller manager 节点和 Gazebo 仿真环境。
具体的控制器激活应该在调用此 launch 文件的应用中处理。

使用方法:
ros2 launch robot_common_launch controller_manager.launch.py robot:=cr5 type:=x5
ros2 launch robot_common_launch controller_manager.launch.py robot:=cr5 hardware:=gz
ros2 launch robot_common_launch controller_manager.launch.py robot:=cr5 hardware:=gz world:=warehouse

# Gazebo GUI可以随时连接（在另一个终端运行）:
gz sim -g
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

# Import robot_common_launch utilities
from robot_common_launch import (
    load_robot_config,
    get_robot_package_path,
    get_gz_bridge_config_path,
    get_gz_image_bridge_topics,
    get_ros2_control_robot_description,
    create_rmw_zenohd_node,
    create_robot_profile_launch_arguments,
    resolve_profile_path,
    resolve_control_sides,
    resolve_control_patch,
    is_compose_asymmetric,
    load_robot_profile,
    write_temp_ros2_control_yaml,
)


def generate_launch_description():
    # 声明参数
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='cr5',
        description='Robot name'
    )
    
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='',
        description='Robot type/variant'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
    
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='dart',
        description='Gazebo world file name (without .sdf extension)'
    )
    
    world_package_arg = DeclareLaunchArgument(
        'world_package',
        default_value='robot_common_launch',
        description='Package containing world files'
    )
    
    
    hardware_arg = DeclareLaunchArgument(
        'hardware',
        default_value='mock_components',
        description='Hardware type: gz for Gazebo, isaac for Isaac, mock_components for mock'
    )

    ctrl_mode_arg = DeclareLaunchArgument(
        "ctrl_mode",
        default_value="2",
        description="2 for position ctrl, 3 for joint impedance, 4 for cartesian ctrl"
    )

    # Remappings parameter (optional, format: "from:to;from2:to2")
    remappings_arg = DeclareLaunchArgument(
        'remappings',
        default_value='',
        description='Topic remappings for ros2_control_node (format: "from1:to1;from2:to2")'
    )

    ocs2_planning_param_file_arg = DeclareLaunchArgument(
        'ocs2_planning_param_file',
        default_value='',
        description='Optional YAML with ocs2_*_controller planning_urdf_* (loaded before controller init)',
    )

    ros2_controllers_override_arg = DeclareLaunchArgument(
        'ros2_controllers_override',
        default_value='',
        description='Pre-merged ros2_control YAML from parent launch (avoids duplicate load/merge)',
    )

    def launch_setup(context, *args, **kwargs):
        configs = context.launch_configurations
        robot_name = configs['robot']
        robot_type = configs['type']
        _use_sim_raw = configs['use_sim_time'].strip().lower()
        use_sim_time = _use_sim_raw in ('true', '1', 'yes')
        world = configs['world']
        world_package = configs['world_package']
        hardware = configs['hardware']
        remappings_str = configs.get('remappings', '')

        robot_profile_path = resolve_profile_path(configs)
        profile = load_robot_profile(robot_profile_path) if robot_profile_path else {}
        control_left, control_right = resolve_control_sides(configs, profile)
        control_patch = resolve_control_patch(profile)
        asymmetric = is_compose_asymmetric(control_left, control_right)

        use_gazebo = hardware == 'gz'

        robot_pkg_path = get_robot_package_path(robot_name)
        if robot_pkg_path is None:
            print(f"[ERROR] Cannot create robot description without package path for robot '{robot_name}'")
            return []

        if use_gazebo:
            print("[INFO] Gazebo mode enabled")

        robot_description = get_ros2_control_robot_description(
            robot_name,
            robot_type=robot_type,
            hardware=hardware,
            launch_configurations=configs,
            robot_profile=robot_profile_path or None,
        )
        
        if robot_description is None:
            print(f"[ERROR] Failed to generate robot_description for '{robot_name}'")
            return []
        
        nodes = []
        
        # 如果使用rmw_zenoh_cpp，自动添加rmw_zenohd节点
        rmw_zenohd_node = create_rmw_zenohd_node()
        if rmw_zenohd_node is not None:
            nodes.append(rmw_zenohd_node)  # 将rmw_zenohd放在最前面，确保先启动
        
        # Robot State Publisher (总是需要)
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                    'robot_description': robot_description,
                    # 必须与 ros2_control_node / rviz2 一致，否则 joint_states 为仿真时间戳时
                    # TF 与 RViz MessageFilter 会出现 “timestamp earlier than transform cache” 等混钟问题。
                    'use_sim_time': use_sim_time,
                }
            ],
        )
        nodes.append(robot_state_publisher)
        
        if use_gazebo:
            # Import Gazebo-related modules only when needed
            from ros_gz_bridge.actions import RosGzBridge
            from ros_gz_sim.actions import GzServer
            
            # 世界文件路径
            world_path = os.path.join(get_package_share_directory(world_package), 'worlds', world + '.sdf')
            
            # 启动 Gazebo Server (创建组合容器)
            gz_server = GzServer(
                world_sdf_file=world_path,
                world_sdf_string='',
                container_name='ros_gz_container',
                create_own_container=True,
                use_composition=True,
            )
            
            # 获取 Gazebo bridge 配置 (优先使用机器人特定配置，否则使用默认配置)
            gz_bridge_config_path = get_gz_bridge_config_path(robot_name)
            
            # 启动 ROS-Gazebo Bridge (复用已创建的容器)
            ros_gz_bridge = RosGzBridge(
                bridge_name='ros_gz_bridge',
                config_file=gz_bridge_config_path,
                container_name='ros_gz_container',
                create_own_container=False,
                use_composition=True,
            )
            
            nodes.extend([gz_server, ros_gz_bridge])
            
            # 检查是否需要启动 Image Bridge (可选)
            gz_image_topics = get_gz_image_bridge_topics(robot_name)
            if gz_image_topics is not None and len(gz_image_topics) > 0:
                # 为每个图像话题创建独立的 image_bridge 节点
                for topic in gz_image_topics:
                    # 从话题名生成节点名称 (例如: /camera/image -> bridge_gz_ros_camera_image)
                    node_name = f'bridge_gz_ros{topic.replace("/", "_")}'
                    
                    image_bridge_node = Node(
                        package='ros_gz_image',
                        executable='image_bridge',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                        ],
                        arguments=[topic],
                    )
                    nodes.append(image_bridge_node)
            
            # 在 Gazebo 中生成机器人
            gz_spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic',
                    'robot_description',
                    '-name',
                    robot_name,
                    '-allow_renaming',
                    'true',
                ],
            )
            
            nodes.append(gz_spawn_entity)
        else:
            ros2_override = configs.get('ros2_controllers_override', '').strip()
            if ros2_override and os.path.isfile(ros2_override):
                with open(ros2_override, 'r', encoding='utf-8') as handle:
                    ros2_controllers_config = yaml.safe_load(handle) or {}
                ros2_controllers_path = ros2_override
            else:
                ros2_controllers_config, ros2_controllers_path = load_robot_config(
                    robot_name,
                    "ros2_control",
                    robot_type,
                    control_left=control_left,
                    control_right=control_right,
                    control_patch=control_patch,
                )
            if ros2_controllers_path is not None or ros2_controllers_config is not None:
                default_remappings = [
                    ("/controller_manager/robot_description", "/robot_description"),
                ]

                additional_remappings = []
                if remappings_str and remappings_str.strip():
                    try:
                        for remap_pair in remappings_str.split(';'):
                            if ':' in remap_pair:
                                from_topic, to_topic = remap_pair.split(':', 1)
                                additional_remappings.append((from_topic.strip(), to_topic.strip()))
                    except Exception as e:
                        print(f"[WARN] Failed to parse remappings '{remappings_str}': {e}")

                all_remappings = default_remappings + additional_remappings

                config_filename = os.path.basename(ros2_controllers_path or "")
                default_config_filename = "ros2_controllers.yaml"
                is_type_specific_config = bool(
                    ros2_controllers_path and config_filename != default_config_filename
                )

                config_has_robot_type = False
                if ros2_controllers_config is not None:
                    try:
                        config_robot_type = ros2_controllers_config.get('ocs2_arm_controller', {}).get('ros__parameters', {}).get('robot_type')
                        config_has_robot_type = config_robot_type is not None
                    except Exception:
                        pass

                node_parameters = []
                use_merged_dict = asymmetric or bool(control_patch)
                if ros2_override and ros2_controllers_config is not None:
                    node_parameters.append(ros2_override)
                elif use_merged_dict and ros2_controllers_config is not None:
                    merged_config_path = write_temp_ros2_control_yaml(ros2_controllers_config)
                    node_parameters.append(merged_config_path)
                    print("[INFO] Using merged ros2_control config file (compose/patch)")
                else:
                    common_config_path = os.path.join(
                        os.path.dirname(ros2_controllers_path), 'common.yaml'
                    )
                    if os.path.exists(common_config_path):
                        node_parameters.append(common_config_path)
                        print(f"[INFO] Loading common ros2_control defaults from: {common_config_path}")
                    if ros2_controllers_path:
                        node_parameters.append(ros2_controllers_path)

                node_parameters.append({'use_sim_time': use_sim_time})

                planning_param_file = configs.get('ocs2_planning_param_file', '').strip()
                if planning_param_file and os.path.isfile(planning_param_file):
                    node_parameters.append(planning_param_file)

                if not is_type_specific_config and not asymmetric:
                    if robot_type and robot_type.strip():
                        node_parameters.append({'robot_type': robot_type})
                        print(f"[INFO] Using launch arg robot_type '{robot_type}' (fallback to default config)")
                elif not config_has_robot_type and not asymmetric:
                    if robot_type and robot_type.strip():
                        node_parameters.append({'robot_type': robot_type})
                        print(f"[INFO] Using launch arg robot_type '{robot_type}' (type-specific config has no robot_type)")
                elif config_has_robot_type:
                    print(f"[INFO] Using robot_type from config file '{config_filename}'")
                
                ros2_control_node = Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=node_parameters,
                    remappings=all_remappings,
                    output="screen",
                )
                nodes.append(ros2_control_node)
            else:
                print(f"[WARN] No controller config found for robot '{robot_name}', skipping ros2_control_node")
        
        # Joint state broadcaster spawner (在所有模式下都启动)
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
            ],
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
        nodes.append(joint_state_broadcaster_spawner)
        
        return nodes

    return LaunchDescription([
        robot_arg,
        type_arg,
        use_sim_time_arg,
        world_arg,
        world_package_arg,
        hardware_arg,
        remappings_arg,
        ocs2_planning_param_file_arg,
        ros2_controllers_override_arg,
    ] + create_robot_profile_launch_arguments() + [
        OpaqueFunction(function=launch_setup)
    ])
