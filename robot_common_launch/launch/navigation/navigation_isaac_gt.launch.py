import os

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

from robot_common_launch.common.navigation_config_utils import (
    default_map_yaml,
    default_nav2_params_yaml,
    resolve_map_yaml,
    resolve_nav2_params,
)


def generate_launch_description():
    common_share = get_package_share_directory('robot_common_launch')
    default_params_yaml = default_nav2_params_yaml()

    default_map_folder = 'warehouse_with_forklifts'
    _default_map_yaml = default_map_yaml()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot = LaunchConfiguration('robot', default='')
    map_config_package = LaunchConfiguration('map_config_package', default='')
    nav_config_package = LaunchConfiguration('nav_config_package', default='')
    params_file = LaunchConfiguration('params_file', default='')
    nav2_profile = LaunchConfiguration('nav2_profile', default='default')
    map_arg = LaunchConfiguration('map')
    resolved_params_file = LaunchConfiguration('resolved_params_file')
    resolved_map_yaml = LaunchConfiguration('resolved_map_yaml')

    def resolve_isaac_gt_launch(context):
        actions = []
        robot_name = robot.perform(context).strip()
        map_pkg = map_config_package.perform(context).strip()
        nav_pkg = nav_config_package.perform(context).strip()
        profile = nav2_profile.perform(context).strip()
        user_params = params_file.perform(context).strip()
        map_value = map_arg.perform(context)

        chosen = resolve_nav2_params(
            robot_name=robot_name,
            nav_config_package=nav_pkg,
            nav2_profile=profile,
            params_file=user_params,
            default_params_yaml=default_params_yaml,
        )
        actions.append(SetLaunchConfiguration('resolved_params_file', chosen))

        resolved_map = resolve_map_yaml(
            map_value,
            robot_name=robot_name,
            map_config_package=map_pkg,
            default_map_yaml_path=_default_map_yaml,
        )
        actions.append(SetLaunchConfiguration('resolved_map_yaml', resolved_map))
        return actions

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
            'yaml_filename': resolved_map_yaml,
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
            default_value=default_map_folder,
            description=(
                'Map folder name under config/maps (prefer map.yaml, else first *.yaml), '
                'or path to .yaml / map directory, or subpath like german_poc/map (.yaml optional); '
                'search order: map_config_package, {robot}_description/config/navigation.yaml, '
                '{robot}_description, robot_common_launch'
            )),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional explicit Nav2 parameter file (highest priority)'),
        DeclareLaunchArgument(
            'nav2_profile',
            default_value='default',
            description=(
                'Nav2 preset when params_file is empty: default=nav2_params_isaac_gt.yaml; '
                'map_only=nav2_params_isaac_gt_map_only.yaml; routing from '
                '{robot}_description/config/navigation.yaml when args empty'
            )),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Robot name without _description suffix, e.g. fiveages_w2'),
        DeclareLaunchArgument(
            'map_config_package',
            default_value='',
            description=(
                'Optional ament package for config/maps '
                '(e.g. fiveages_w2_common_description)'
            )),
        DeclareLaunchArgument(
            'nav_config_package',
            default_value='',
            description=(
                'Optional ament package for config/nav2 '
                '(e.g. fiveages_w2_common_description)'
            )),
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

        OpaqueFunction(function=resolve_isaac_gt_launch),
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
