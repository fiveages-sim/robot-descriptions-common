"""Build controller_manager, robot_state_publisher, and Gazebo stack nodes."""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from .control_compose import is_compose_asymmetric
from .launch_arg_utils import (
    load_robot_profile,
    resolve_control_patch,
    resolve_control_sides,
    resolve_profile_path,
    resolve_robot_variant,
)
from .robot_utils import (
    EMPTY_ROBOT_CONFIG_META,
    RobotConfigMeta,
    get_gz_bridge_config_path,
    get_gz_image_bridge_topics,
    get_robot_package_path,
    get_ros2_control_robot_description,
    load_robot_config,
)
from .launch_utils import create_rmw_zenohd_node


def _parse_remappings(remappings_str: str) -> List[tuple]:
    additional = []
    if remappings_str and remappings_str.strip():
        try:
            for remap_pair in remappings_str.split(";"):
                if ":" in remap_pair:
                    from_topic, to_topic = remap_pair.split(":", 1)
                    additional.append((from_topic.strip(), to_topic.strip()))
        except Exception as exc:
            print(f"[WARN] Failed to parse remappings '{remappings_str}': {exc}")
    return additional


def _build_ros2_control_node_parameters(
    *,
    ros2_controllers_config: Optional[Dict[str, Any]],
    ros2_controllers_path: Optional[str],
    meta: RobotConfigMeta,
    use_sim_time: bool,
    ocs2_planning_param_file: str,
    robot_type: str,
    asymmetric: bool,
) -> List[Any]:
    if ros2_controllers_config is None and ros2_controllers_path is None:
        return []

    config_filename = os.path.basename(ros2_controllers_path or "")
    default_config_filename = "ros2_controllers.yaml"
    is_type_specific_config = bool(
        ros2_controllers_path and config_filename != default_config_filename
    )

    config_has_robot_type = False
    if isinstance(ros2_controllers_config, dict):
        try:
            config_robot_type = (
                ros2_controllers_config.get("ocs2_arm_controller", {})
                .get("ros__parameters", {})
                .get("robot_type")
            )
            config_has_robot_type = config_robot_type is not None
        except Exception:
            pass

    node_parameters: List[Any] = []
    if meta.needs_merged_file and meta.merged_yaml_path:
        node_parameters.append(meta.merged_yaml_path)
        print(f"[INFO] Using merged ros2_control config file: {meta.merged_yaml_path}")
    else:
        common_config_path = os.path.join(
            os.path.dirname(ros2_controllers_path or ""), "common.yaml"
        )
        if os.path.exists(common_config_path):
            node_parameters.append(common_config_path)
            print(f"[INFO] Loading common ros2_control defaults from: {common_config_path}")
        if ros2_controllers_path:
            node_parameters.append(ros2_controllers_path)

    node_parameters.append({"use_sim_time": use_sim_time})

    if ocs2_planning_param_file and os.path.isfile(ocs2_planning_param_file):
        node_parameters.append(ocs2_planning_param_file)

    if not is_type_specific_config and not asymmetric:
        if robot_type and robot_type.strip():
            node_parameters.append({"robot_type": robot_type})
            print(f"[INFO] Using launch arg robot_type '{robot_type}' (fallback to default config)")
    elif not config_has_robot_type and not asymmetric:
        if robot_type and robot_type.strip():
            node_parameters.append({"robot_type": robot_type})
            print(
                f"[INFO] Using launch arg robot_type '{robot_type}' "
                "(type-specific config has no robot_type)"
            )
    elif config_has_robot_type:
        print(f"[INFO] Using robot_type from config file '{config_filename}'")

    return node_parameters


def create_controller_manager_nodes(
    *,
    robot_name: str,
    robot_type: str = "",
    hardware: str = "mock_components",
    use_sim_time: bool = False,
    world: str = "dart",
    world_package: str = "robot_common_launch",
    remappings_str: str = "",
    ocs2_planning_param_file: str = "",
    ros2_controllers_override: str = "",
    launch_configurations: Optional[Dict[str, str]] = None,
    preloaded_config: Optional[Dict[str, Any]] = None,
    preloaded_meta: Optional[RobotConfigMeta] = None,
) -> List[Any]:
    """
    Create robot_state_publisher, optional Gazebo stack, ros2_control_node, and
    joint_state_broadcaster spawner.

    When preloaded_config/meta are provided (OCS2 parent launch), skip reloading
    ros2_control config but still resolve profile for URDF xacro.
    """
    configs = launch_configurations or {}
    robot_profile_path = resolve_profile_path(configs)
    profile = load_robot_profile(robot_profile_path) if robot_profile_path else {}
    control_left, control_right = resolve_control_sides(configs, profile)
    control_patch = resolve_control_patch(profile)
    robot_variant = resolve_robot_variant(configs, profile)
    asymmetric = is_compose_asymmetric(control_left, control_right)

    use_gazebo = hardware == "gz"

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

    nodes: List[Any] = []

    rmw_zenohd_node = create_rmw_zenohd_node()
    if rmw_zenohd_node is not None:
        nodes.append(rmw_zenohd_node)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "publish_frequency": 100.0,
                "use_tf_static": True,
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    nodes.append(robot_state_publisher)

    if use_gazebo:
        from ros_gz_bridge.actions import RosGzBridge
        from ros_gz_sim.actions import GzServer

        world_path = os.path.join(
            get_package_share_directory(world_package), "worlds", world + ".sdf"
        )
        gz_server = GzServer(
            world_sdf_file=world_path,
            world_sdf_string="",
            container_name="ros_gz_container",
            create_own_container=True,
            use_composition=True,
        )
        gz_bridge_config_path = get_gz_bridge_config_path(robot_name)
        ros_gz_bridge = RosGzBridge(
            bridge_name="ros_gz_bridge",
            config_file=gz_bridge_config_path,
            container_name="ros_gz_container",
            create_own_container=False,
            use_composition=True,
        )
        nodes.extend([gz_server, ros_gz_bridge])

        gz_image_topics = get_gz_image_bridge_topics(robot_name)
        if gz_image_topics:
            for topic in gz_image_topics:
                node_name = f"bridge_gz_ros{topic.replace('/', '_')}"
                nodes.append(
                    Node(
                        package="ros_gz_image",
                        executable="image_bridge",
                        name=node_name,
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                        arguments=[topic],
                    )
                )

        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    robot_name,
                    "-allow_renaming",
                    "true",
                ],
            )
        )
    else:
        meta = preloaded_meta or EMPTY_ROBOT_CONFIG_META
        ros2_controllers_config = preloaded_config
        ros2_controllers_path: Optional[str] = None

        override = (ros2_controllers_override or meta.merged_yaml_path or "").strip()
        if override and os.path.isfile(override):
            with open(override, "r", encoding="utf-8") as handle:
                ros2_controllers_config = yaml.safe_load(handle) or {}
            ros2_controllers_path = override
            if not meta.merged_yaml_path:
                meta = RobotConfigMeta(
                    compose_applied=meta.compose_applied,
                    type_yaml_found=meta.type_yaml_found,
                    patch_applied=meta.patch_applied,
                    variant_overlay_applied=meta.variant_overlay_applied,
                    needs_merged_file=True,
                    merged_yaml_path=override,
                    base_config_path=meta.base_config_path,
                    hardware_overlay_applied=meta.hardware_overlay_applied,
                )
        elif preloaded_config is not None:
            ros2_controllers_path = meta.base_config_path or meta.merged_yaml_path or None
        else:
            ros2_controllers_config, ros2_controllers_path, meta = load_robot_config(
                robot_name,
                "ros2_control",
                robot_type,
                control_left=control_left,
                control_right=control_right,
                control_patch=control_patch,
                robot_variant=robot_variant,
                hardware=hardware,
            )

        node_parameters = _build_ros2_control_node_parameters(
            ros2_controllers_config=ros2_controllers_config,
            ros2_controllers_path=ros2_controllers_path,
            meta=meta,
            use_sim_time=use_sim_time,
            ocs2_planning_param_file=ocs2_planning_param_file,
            robot_type=robot_type,
            asymmetric=asymmetric,
        )
        if node_parameters:
            default_remappings = [
                ("/controller_manager/robot_description", "/robot_description"),
            ]
            all_remappings = default_remappings + _parse_remappings(remappings_str)
            nodes.append(
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=node_parameters,
                    remappings=all_remappings,
                    output="screen",
                )
            )
        else:
            print(f"[WARN] No controller config found for robot '{robot_name}', skipping ros2_control_node")

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )
    )

    return nodes
