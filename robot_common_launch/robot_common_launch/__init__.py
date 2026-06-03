# robot_common_launch package
# Common launch utilities for robots

from .common.robot_utils import (
    get_robot_package_path,
    load_robot_config,
    get_info_file_name,
    get_gz_bridge_config_path,
    get_gz_image_bridge_topics,
    clear_config_cache,
    clear_robot_description_cache,
    get_ros2_control_robot_description,
    get_planning_robot_description,
    build_planning_urdf_launch_params,
    write_temp_ros2_control_yaml,
    parse_task_info,
    prepare_arms_target_manager_parameters,
)
from .common.controller_utils import detect_controllers, create_controller_spawners
from .common.launch_arg_utils import (
    build_xacro_mappings,
    forward_robot_launch_args,
    create_robot_profile_launch_arguments,
    resolve_profile_path,
    resolve_control_sides,
    resolve_control_patch,
    should_use_base_planning_urdf,
    load_robot_profile,
)
from .common.control_compose import compose_control_config, resolve_compose_type_key
from .common.launch_utils import (
    process_xacro, 
    get_default_type_from_xacro,
    create_visualization_nodes,
    create_common_launch_arguments,
    create_visualization_launch_description,
    is_rmw_zenoh,
    create_rmw_zenohd_node,
    parse_launch_mode,
    create_launch_mode_arguments
)

__all__ = [
    'get_robot_package_path',
    'load_robot_config',
    'get_info_file_name',
    'get_gz_bridge_config_path',
    'get_gz_image_bridge_topics',
    'clear_config_cache',
    'clear_robot_description_cache',
    'get_ros2_control_robot_description',
    'get_planning_robot_description',
    'build_planning_urdf_launch_params',
    'write_temp_ros2_control_yaml',
    'build_xacro_mappings',
    'forward_robot_launch_args',
    'create_robot_profile_launch_arguments',
    'resolve_profile_path',
    'resolve_control_sides',
    'resolve_control_patch',
    'should_use_base_planning_urdf',
    'load_robot_profile',
    'compose_control_config',
    'resolve_compose_type_key',
    'parse_task_info',
    'prepare_arms_target_manager_parameters',
    'detect_controllers', 
    'create_controller_spawners',
    'process_xacro',
    'get_default_type_from_xacro',
    'create_visualization_nodes',
    'create_common_launch_arguments',
    'create_visualization_launch_description',
    'is_rmw_zenoh',
    'create_rmw_zenohd_node',
    'parse_launch_mode',
    'create_launch_mode_arguments'
]
