import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    default_params_yaml = os.path.join(
        get_package_share_directory('robot_common_launch'),
        'config', 'nav2', 'nav2_params_isaac_gt.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_map_yaml = os.path.join(
        get_package_share_directory('robot_common_launch'),
        'config', 'maps', 'warehouse_with_forklifts', 'maps.yaml')
    map_yaml = LaunchConfiguration(
        'map',
        default=default_map_yaml)
    robot = LaunchConfiguration('robot', default='')
    params_file = LaunchConfiguration('params_file', default='')
    resolved_params_file = LaunchConfiguration('resolved_params_file')
    def resolve_params_file(context):
        user_params = params_file.perform(context).strip()
        if user_params:
            return [SetLaunchConfiguration('resolved_params_file', user_params)]

        robot_name = robot.perform(context).strip()
        if robot_name:
            robot_pkg = f'{robot_name}_description'
            try:
                robot_share = get_package_share_directory(robot_pkg)
                robot_params = os.path.join(
                    robot_share, 'config', 'nav2', 'nav2_params_isaac_gt.yaml')
                if os.path.exists(robot_params):
                    return [SetLaunchConfiguration('resolved_params_file', robot_params)]
            except PackageNotFoundError:
                pass

        return [SetLaunchConfiguration('resolved_params_file', default_params_yaml)]


    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')
    publish_map_odom_tf = LaunchConfiguration('publish_map_odom_tf', default='false')
    publish_map_world_tf = LaunchConfiguration('publish_map_world_tf', default='true')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    nav2_navigation = GroupAction(actions=[
        SetRemap(src='/scan', dst=scan_topic),
        SetRemap(src='scan', dst=scan_topic),
        SetRemap(src='/odom', dst=odom_topic),
        SetRemap(src='odom', dst=odom_topic),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': resolved_params_file,
            }.items(),
        ),
    ])

    # 必须与 Nav2 / Isaac 一致使用仿真时钟；否则 TF 用 wall time、其它节点用 /clock，
    # 会出现 map<->odom 与代价地图/点云时间差数十秒，报 Transform too old / 点云被丢弃。
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_map_odom_tf),
    )
    map_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_map_world_tf),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional explicit Nav2 parameter file (highest priority)'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Robot name without _description suffix, e.g. fiveages_w1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic from Isaac Sim'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Ground truth odometry topic from Isaac Sim'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame id'),
        DeclareLaunchArgument(
            'publish_map_odom_tf',
            default_value='false',
            description='Optionally publish static map->odom TF'),
        DeclareLaunchArgument(
            'publish_map_world_tf',
            default_value='true',
            description='Optionally publish static map->world TF'),

        OpaqueFunction(function=resolve_params_file),
        map_to_odom_tf,
        map_to_world_tf,
        map_server_node,
        lifecycle_manager_map,
        nav2_navigation,

        Node(
            package='rviz2',
            executable='rviz2',
            name='nav2_rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
