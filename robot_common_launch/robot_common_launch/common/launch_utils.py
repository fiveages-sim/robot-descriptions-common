"""
通用 launch 工具函数

这个模块提供了用于 launch 文件的通用工具函数，用于消除重复代码。
"""

import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro


def get_default_type_from_xacro(xacro_file):
    """
    从 xacro 文件中读取 type 参数的默认值
    
    Args:
        xacro_file (str): xacro 文件路径
        
    Returns:
        str: 默认的 type 值，如果找不到则返回 None
    """
    try:
        with open(xacro_file, 'r') as f:
            content = f.read()
            # 查找 xacro:arg name="type" default="..." 的模式
            match = re.search(r'xacro:arg\s+name=["\']type["\']\s+default=["\']([^"\']+)["\']', content)
            if match:
                return match.group(1)
    except Exception as e:
        print(f"Warning: Could not read default type from {xacro_file}: {e}")
    return None


def process_xacro(robot_name, xacro_filename="robot.xacro", **kwargs):
    """
    通用的 xacro 处理函数
    
    Args:
        robot_name (str): 机器人名称
        xacro_filename (str): xacro 文件名 (默认: "robot.xacro")
        **kwargs: 传递给 xacro 的映射参数 (如 type, collider, direction 等)
        
    Returns:
        str: 处理后的 URDF XML 字符串
    """
    package_description = robot_name + "_description"
    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, 'xacro', xacro_filename)
    
    # 构建 mappings 字典，过滤空值
    mappings = {k: v for k, v in kwargs.items() if v and str(v).strip()}
    
    # 特殊处理：如果 type 参数为空，尝试从 xacro 文件读取默认值
    if 'type' not in mappings:
        default_type = get_default_type_from_xacro(xacro_file)
        if default_type:
            mappings['type'] = default_type
    
    # 根据 mappings 是否为空来决定如何处理 xacro 文件
    if mappings:
        robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    else:
        robot_description_config = xacro.process_file(xacro_file)
    
    return robot_description_config.toxml()


def create_visualization_nodes(robot_description, rviz_config_file):
    """
    创建通用的可视化节点
    
    Args:
        robot_description (str): 机器人描述 XML
        rviz_config_file (str): RViz 配置文件路径
        
    Returns:
        list: Node 对象列表
    """
    return [
        # rviz2 executable hosts multiple rclcpp nodes; avoid name= remaps to one name.
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                    'robot_description': robot_description
                }
            ],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
        )
    ]


def is_rmw_zenoh():
    """
    检测当前使用的ROS2中间件是否为rmw_zenoh_cpp
    
    Returns:
        bool: 如果使用rmw_zenoh_cpp则返回True，否则返回False
    """
    rmw_implementation = os.environ.get('RMW_IMPLEMENTATION', '')
    return rmw_implementation == 'rmw_zenoh_cpp'


def is_zenoh_router_running():
    """
    检测系统中是否已经有zenoh router在运行
    
    Returns:
        bool: 如果已经有zenoh router在运行则返回True，否则返回False
    """
    try:
        # 检查是否有rmw_zenohd进程在运行
        result = subprocess.run(
            ['pgrep', '-f', 'rmw_zenohd'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            return True
        
        # 也检查是否有zenohd进程在运行（独立的zenoh router）
        result = subprocess.run(
            ['pgrep', '-f', 'zenohd'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            return True
        
        return False
    except Exception as e:
        # 如果检测失败（例如pgrep命令不存在），默认返回False，允许启动
        print(f"[WARNING] Failed to check if zenoh router is running: {e}")
        return False


def create_rmw_zenohd_node():
    """
    创建rmw_zenohd节点（当使用rmw_zenoh_cpp中间件时）
    如果系统中已经有zenoh router在运行，则不会重复启动
    
    Returns:
        Node: rmw_zenohd节点对象，如果不需要或已存在则返回None
    """
    if not is_rmw_zenoh():
        return None
    
    # 检查是否已经有zenoh router在运行
    if is_zenoh_router_running():
        print("[INFO] Zenoh router is already running, skipping rmw_zenohd node creation")
        return None
    
    return Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='rmw_zenohd',
        output='screen',
    )


def create_common_launch_arguments():
    """
    创建通用的 launch 参数
    
    Returns:
        list: DeclareLaunchArgument 对象列表
    """
    from .launch_arg_utils import create_eef_side_launch_arguments

    return [
        DeclareLaunchArgument(
            'type',
            default_value='',
            description='Type parameter for xacro (empty means no type parameter passed to xacro)'
        ),
        *create_eef_side_launch_arguments(),
        DeclareLaunchArgument(
            'collider',
            default_value='',
            description='Collider type parameter for xacro (empty means no collider parameter passed to xacro)'
        ),
        DeclareLaunchArgument(
            'direction',
            default_value='',
            description='Direction parameter for xacro (empty means no direction parameter passed to xacro)'
        ),
    ]


def build_visualization_xacro_mappings(robot_name: str, launch_configurations: dict) -> dict:
    """Xacro mappings for RViz-only launches (no ros2_control / robot_profile)."""
    from .launch_arg_utils import _cli_launch_value, resolve_side_eef_types

    mappings = {}
    for key in ("collider", "direction"):
        value = launch_configurations.get(key, "")
        if value and str(value).strip():
            mappings[key] = str(value).strip()

    launch_type = _cli_launch_value(launch_configurations, "type")
    side_left, side_right = resolve_side_eef_types(launch_configurations, None)
    if launch_type:
        mappings["type"] = launch_type
    if side_left:
        mappings["left_type"] = side_left
    if side_right:
        mappings["right_type"] = side_right
    return mappings


def create_visualization_launch_description(
    robot_param_name='robot',
    robot_default_value='go1',
    xacro_filename='robot.xacro',
    rviz_config_name='urdf.rviz',
    additional_args=None
):
    """
    创建通用的可视化 launch 描述
    
    Args:
        robot_param_name (str): 机器人参数名称
        robot_default_value (str): 机器人默认值
        xacro_filename (str): xacro 文件名
        rviz_config_name (str): RViz 配置文件名
        additional_args (list): 额外的参数列表
        
    Returns:
        LaunchDescription: 配置好的 launch 描述
    """
    def launch_setup(context, *args, **kwargs):
        robot_value = context.launch_configurations[robot_param_name]
        mappings = build_visualization_xacro_mappings(
            robot_value, context.launch_configurations
        )

        package_description = robot_value + "_description"
        pkg_path = get_package_share_directory(package_description)
        xacro_file = os.path.join(pkg_path, "xacro", xacro_filename)
        if "type" not in mappings:
            default_type = get_default_type_from_xacro(xacro_file)
            if default_type:
                mappings["type"] = default_type

        robot_description = xacro.process_file(xacro_file, mappings=mappings).toxml()
        
        # 获取 RViz 配置文件
        rviz_config_file = os.path.join(
            get_package_share_directory("robot_common_launch"), 
            "config", "rviz", rviz_config_name
        )
        
        # 创建可视化节点
        nodes = create_visualization_nodes(robot_description, rviz_config_file)
        
        # 如果使用rmw_zenoh_cpp，自动添加rmw_zenohd节点
        rmw_zenohd_node = create_rmw_zenohd_node()
        if rmw_zenohd_node is not None:
            nodes.insert(0, rmw_zenohd_node)  # 将rmw_zenohd放在最前面，确保先启动
        
        return nodes

    # 构建参数列表
    args = [
        DeclareLaunchArgument(
            robot_param_name,
            default_value=robot_default_value,
            description=f'{robot_param_name} name to visualize'
        )
    ]
    
    # 添加通用参数
    args.extend(create_common_launch_arguments())

    # 添加额外参数
    if additional_args:
        args.extend(additional_args)
    
    # 添加 opaque function
    args.append(OpaqueFunction(function=launch_setup))
    
    return LaunchDescription(args)


def parse_launch_mode(context):
    """
    解析launch_mode配置
    
    Args:
        context: Launch context对象
        
    Returns:
        tuple: (launch_mode, rviz_only, use_rviz)
            - launch_mode: 'full', 'control_only', or 'rviz_only'
            - rviz_only: bool, 是否为rviz_only模式
            - use_rviz: bool, 是否需要启动rviz
    """
    launch_mode = context.launch_configurations.get('launch_mode', 'full').lower()
    rviz_only = launch_mode == 'rviz_only'
    use_rviz = launch_mode in ['full', 'rviz_only']
    
    return launch_mode, rviz_only, use_rviz


def create_launch_mode_arguments():
    """
    创建launch_mode相关的参数声明
    
    Returns:
        list: DeclareLaunchArgument对象列表
    """
    return [
        DeclareLaunchArgument(
            'launch_mode',
            default_value='full',
            description="Launch mode: 'full' (rviz + control, default), 'control_only' (no rviz), or 'rviz_only' (only rviz for remote visualization)"
        ),
    ]


