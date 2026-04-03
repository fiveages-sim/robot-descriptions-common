import glob
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


def _find_map_yaml_in_directory(directory: str):
    """Prefer map.yaml; otherwise first *.yaml in directory (lexicographic)."""
    if not os.path.isdir(directory):
        return None
    primary = os.path.join(directory, 'map.yaml')
    if os.path.isfile(primary):
        return os.path.abspath(primary)
    yamls = sorted(glob.glob(os.path.join(directory, '*.yaml')))
    if yamls:
        return os.path.abspath(yamls[0])
    return None


def _resolve_map_yaml(map_arg: str, robot_name: str, default_map_yaml: str, common_share: str) -> str:
    """
    Resolve map for map_server: absolute yaml path, or directory, or name / relative path under
    {robot}_description/config/maps then robot_common_launch/config/maps, else default_map_yaml.
    """
    raw = map_arg.strip()
    if not raw:
        return default_map_yaml

    expanded = os.path.expanduser(raw)

    if os.path.isfile(expanded):
        return os.path.abspath(expanded)

    if os.path.isdir(expanded):
        found = _find_map_yaml_in_directory(expanded)
        if found:
            return found

    maps_roots = []
    if robot_name:
        try:
            robot_share = get_package_share_directory(f'{robot_name}_description')
            maps_roots.append(os.path.join(robot_share, 'config', 'maps'))
        except PackageNotFoundError:
            pass
    maps_roots.append(os.path.join(common_share, 'config', 'maps'))

    for root in maps_roots:
        candidate = os.path.normpath(os.path.join(root, expanded))
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
        if os.path.isdir(candidate):
            found = _find_map_yaml_in_directory(candidate)
            if found:
                return found
        # 允许写 german_poc/map 这类「子路径无后缀」，自动补 .yaml
        if not expanded.endswith(('.yaml', '.yml')):
            yaml_only = candidate + '.yaml'
            if os.path.isfile(yaml_only):
                return os.path.abspath(yaml_only)

    hint = f'{robot_name}_description/config/maps, then ' if robot_name else ''
    print(
        f'[navigation_isaac_gt] Could not resolve map {raw!r} under '
        f'{hint}robot_common_launch/config/maps; using default:\n  {default_map_yaml}'
    )
    return default_map_yaml


def generate_launch_description():
    common_share = get_package_share_directory('robot_common_launch')
    default_params_yaml = os.path.join(
        common_share, 'config', 'nav2', 'nav2_params_isaac_gt.yaml')

    # map 参数默认只写「地图文件夹名」；在 config/maps/<文件夹>/ 下优先 map.yaml，否则任取 *.yaml
    default_map_folder = 'warehouse_with_forklifts'
    _default_map_dir = os.path.join(
        common_share, 'config', 'maps', default_map_folder)
    default_map_yaml = _find_map_yaml_in_directory(_default_map_dir)
    if not default_map_yaml:
        default_map_yaml = os.path.join(_default_map_dir, 'map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot = LaunchConfiguration('robot', default='')
    params_file = LaunchConfiguration('params_file', default='')
    map_arg = LaunchConfiguration('map')
    resolved_params_file = LaunchConfiguration('resolved_params_file')
    resolved_map_yaml = LaunchConfiguration('resolved_map_yaml')

    def resolve_isaac_gt_launch(context):
        actions = []

        user_params = params_file.perform(context).strip()
        if user_params:
            actions.append(SetLaunchConfiguration('resolved_params_file', user_params))
        else:
            robot_name = robot.perform(context).strip()
            chosen = default_params_yaml
            if robot_name:
                try:
                    robot_share = get_package_share_directory(f'{robot_name}_description')
                    robot_params = os.path.join(
                        robot_share, 'config', 'nav2', 'nav2_params_isaac_gt.yaml')
                    if os.path.exists(robot_params):
                        chosen = robot_params
                except PackageNotFoundError:
                    pass
            actions.append(SetLaunchConfiguration('resolved_params_file', chosen))

        robot_name = robot.perform(context).strip()
        map_value = map_arg.perform(context)
        resolved_map = _resolve_map_yaml(
            map_value, robot_name, default_map_yaml, common_share)
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
            default_value=default_map_folder,
            description=(
                'Map folder name under config/maps (prefer map.yaml, else first *.yaml), '
                'or path to .yaml / map directory, or subpath like german_poc/map (.yaml optional); '
                'robot package config/maps is tried before robot_common_launch'
            )),
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
