"""
Common controller utilities for launch files.

This module provides utility functions for controller detection and management.
"""

import xml.etree.ElementTree as ET
from launch_ros.actions import Node


def _extract_joints_from_urdf(robot_description):
    """
    从 URDF/XML 字符串中提取所有 joint 名称。
    
    Args:
        robot_description (str): URDF/XML 格式的机器人描述字符串
        
    Returns:
        set: 所有 joint 名称的集合
    """
    if not robot_description:
        return set()
    
    try:
        root = ET.fromstring(robot_description)
        joints = set()
        for joint in root.findall('.//joint'):
            name_attr = joint.get('name')
            if name_attr:
                joints.add(name_attr)
        return joints
    except ET.ParseError as e:
        print(f"[WARN] Failed to parse robot_description XML: {e}")
        return set()
    except Exception as e:
        print(f"[WARN] Error extracting joints from robot_description: {e}")
        return set()


def detect_controllers(robot_name, robot_type="", patterns=None, robot_description=None):
    """
    Detect controllers from ROS2 controller configuration.
    
    如果提供了 robot_description，会检查控制器配置中指定的 joint 是否真的存在于 xacro 中。
    只有 joint 存在的控制器才会被返回。
    
    Args:
        robot_name (str): Name of the robot
        robot_type (str): Robot type/variant
        patterns (list): List of patterns to match controller names
        robot_description (str, optional): URDF/XML 格式的机器人描述字符串，用于验证 joint 是否存在
        
    Returns:
        list: List of detected controllers (filtered by joint existence if robot_description is provided)
        
    Example:
        >>> controllers = detect_controllers('cr5', 'x5', ['hand', 'gripper'])
        >>> print(controllers)
        [{'name': 'hand_controller', 'type': 'joint_trajectory_controller', ...}]
        
        >>> # 带 joint 验证
        >>> controllers = detect_controllers('cr5', 'x5', ['hand'], robot_description=xml_string)
    """
    # Import here to avoid circular imports
    from .robot_utils import load_robot_config
    
    if patterns is None:
        patterns = ['hand', 'gripper']
    
    config, _ = load_robot_config(robot_name, "ros2_control", robot_type)
    
    if config is None:
        print(f"[WARN] No controllers will be detected for robot '{robot_name}'")
        return []
    
    # 如果提供了 robot_description，提取所有 joint 名称
    available_joints = set()
    if robot_description:
        available_joints = _extract_joints_from_urdf(robot_description)
        if available_joints:
            print(f"[INFO] Found {len(available_joints)} joints in robot_description for validation")
    
    controllers = []
    # 记录被跳过的 controller 名称，避免在后续检测中重复处理
    skipped_controllers = set()
    
    # Check controller_manager section for matching controllers
    controller_manager = config.get('controller_manager', {}).get('ros__parameters', {})
    
    for controller_name, controller_config in controller_manager.items():
        # Check if controller name matches any pattern
        if any(pattern.lower() in controller_name.lower() for pattern in patterns):
            # Extract controller type and parameters
            controller_type = controller_config.get('type', '')
            
            # 如果提供了 robot_description，检查配置中的 joint 是否存在
            if robot_description and available_joints:
                # 检查控制器配置中的 joint 字段
                controller_params = config.get(controller_name, {}).get('ros__parameters', {})
                joint_name = controller_params.get('joint')
                
                if joint_name:
                    if joint_name not in available_joints:
                        print(f"[WARN] Controller '{controller_name}' specifies joint '{joint_name}' which does not exist in robot_description, skipping")
                        skipped_controllers.add(controller_name)
                        continue
                    else:
                        print(f"[INFO] Detected controller: {controller_name} ({controller_type}) with joint '{joint_name}' (verified)")
                else:
                    # 如果没有指定 joint，仍然添加（可能是其他类型的控制器）
                    print(f"[INFO] Detected controller: {controller_name} ({controller_type}) (no joint specified)")
            else:
                # 没有提供 robot_description，直接添加
                print(f"[INFO] Detected controller: {controller_name} ({controller_type})")
            
            controllers.append({
                'name': controller_name,
                'type': controller_type,
                'config': controller_config
            })
    
    # Also check for controller parameter sections
    for section_name, section_config in config.items():
        if any(pattern.lower() in section_name.lower() for pattern in patterns):
            # 检查是否已经在 controllers 列表中，或者已经被跳过
            if section_name in [c['name'] for c in controllers] or section_name in skipped_controllers:
                continue
            
            # 如果提供了 robot_description，检查配置中的 joint 是否存在
            if robot_description and available_joints:
                # 检查控制器配置中的 joint 字段
                section_params = section_config.get('ros__parameters', {})
                joint_name = section_params.get('joint')
                
                if joint_name:
                    if joint_name not in available_joints:
                        print(f"[WARN] Controller section '{section_name}' specifies joint '{joint_name}' which does not exist in robot_description, skipping")
                        skipped_controllers.add(section_name)
                        continue
                    else:
                        print(f"[INFO] Detected controller section: {section_name} with joint '{joint_name}' (verified)")
                else:
                    # 如果没有指定 joint，仍然添加
                    print(f"[INFO] Detected controller section: {section_name} (no joint specified)")
            else:
                print(f"[INFO] Detected controller section: {section_name}")
            
            controllers.append({
                'name': section_name,
                'type': 'unknown',
                'config': section_config
            })
    
    print(f"[INFO] Total controllers detected: {len(controllers)}")
    return controllers


def create_controller_spawners(controllers, use_sim_time=False):
    """
    Create spawner nodes for controllers.
    
    Args:
        controllers (list): List of controller configurations
        use_sim_time (bool): Whether to use simulation time
        
    Returns:
        list: List of Node objects for controller spawners
        
    Example:
        >>> controllers = detect_controllers('cr5', 'x5', ['hand'])
        >>> spawners = create_controller_spawners(controllers, use_sim_time=True)
        >>> print(len(spawners))
        1
    """
    spawners = []
    
    for controller in controllers:
        controller_name = controller['name']
        
        print(f"[INFO] Creating spawner for controller: {controller_name}")
        
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name],
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )
        
        spawners.append(spawner)
    
    return spawners
