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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from robot_common_launch import (
    create_controller_manager_nodes,
    create_robot_profile_launch_arguments,
    load_robot_profile,
    resolve_control_patch,
    resolve_control_sides,
    resolve_profile_path,
)


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="cr5",
        description="Robot name",
    )

    type_arg = DeclareLaunchArgument(
        "type",
        default_value="",
        description="Robot type/variant",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether to use simulation time",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="dart",
        description="Gazebo world file name (without .sdf extension)",
    )

    world_package_arg = DeclareLaunchArgument(
        "world_package",
        default_value="robot_common_launch",
        description="Package containing world files",
    )

    hardware_arg = DeclareLaunchArgument(
        "hardware",
        default_value="mock_components",
        description="Hardware type: gz for Gazebo, isaac for Isaac, mock_components for mock",
    )

    remappings_arg = DeclareLaunchArgument(
        "remappings",
        default_value="",
        description='Topic remappings for ros2_control_node (format: "from1:to1;from2:to2")',
    )

    ocs2_planning_param_file_arg = DeclareLaunchArgument(
        "ocs2_planning_param_file",
        default_value="",
        description="Optional YAML with ocs2_*_controller planning_urdf_* (loaded before controller init)",
    )

    ros2_controllers_override_arg = DeclareLaunchArgument(
        "ros2_controllers_override",
        default_value="",
        description="Pre-merged ros2_control YAML from parent launch (avoids duplicate load/merge)",
    )

    def launch_setup(context, *args, **kwargs):
        configs = context.launch_configurations
        _use_sim_raw = configs["use_sim_time"].strip().lower()
        use_sim_time = _use_sim_raw in ("true", "1", "yes")

        return create_controller_manager_nodes(
            robot_name=configs["robot"],
            robot_type=configs.get("type", ""),
            hardware=configs.get("hardware", "mock_components"),
            use_sim_time=use_sim_time,
            world=configs.get("world", "dart"),
            world_package=configs.get("world_package", "robot_common_launch"),
            remappings_str=configs.get("remappings", ""),
            ocs2_planning_param_file=configs.get("ocs2_planning_param_file", "").strip(),
            ros2_controllers_override=configs.get("ros2_controllers_override", "").strip(),
            launch_configurations=configs,
        )

    return LaunchDescription(
        [
            robot_arg,
            type_arg,
            use_sim_time_arg,
            world_arg,
            world_package_arg,
            hardware_arg,
            remappings_arg,
            ocs2_planning_param_file_arg,
            ros2_controllers_override_arg,
        ]
        + create_robot_profile_launch_arguments()
        + [OpaqueFunction(function=launch_setup)]
    )
