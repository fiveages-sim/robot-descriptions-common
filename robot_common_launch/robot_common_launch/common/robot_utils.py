"""
Common robot utilities for launch files.

This module provides utility functions for robot path management and configuration loading.
"""

import os
import re
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

# 全局缓存字典，避免重复读取配置文件
_config_cache = {}

# 全局缓存字典，避免重复生成 robot_description
_robot_description_cache = {}


def clear_config_cache():
    """
    清除配置缓存
    
    在开发或测试时，如果配置文件被修改，可以调用此函数清除缓存
    """
    global _config_cache
    _config_cache.clear()
    print("[INFO] Config cache cleared")


def clear_robot_description_cache():
    """
    清除 robot_description 缓存
    
    在开发或测试时，如果 xacro 文件被修改，可以调用此函数清除缓存
    """
    global _robot_description_cache
    _robot_description_cache.clear()
    print("[INFO] Robot description cache cleared")


def get_robot_package_path(robot_name):
    """
    Get common robot-related paths.
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'arx5', etc.)
        
    Returns:
        str: Path to the robot description package, or None if not found
        
    Example:
        >>> robot_path = get_robot_package_path('cr5')
        >>> print(robot_path)
        '/opt/ros/humble/share/cr5_description'
    """
    robot_pkg = robot_name + "_description"
    try:
        robot_pkg_path = get_package_share_directory(robot_pkg)
        return robot_pkg_path
    except Exception as e:
        print(f"[ERROR] Failed to get package path for '{robot_pkg}': {e}")
        return None


def _generate_progressive_type_candidates(robot_type):
    """
    生成渐进匹配的type候选列表。
    
    例如，对于'ccs_left_rg75'，会生成：
    ['ccs_left_rg75', 'ccs_left', 'ccs']
    
    Args:
        robot_type (str): 完整的robot type字符串
        
    Returns:
        list: 渐进缩短的type候选列表
    """
    if not robot_type or not robot_type.strip():
        return []
    
    candidates = []
    current_type = robot_type.strip()
    
    # 添加完整名称
    candidates.append(current_type)
    
    # 逐步缩短：每次去掉最后一个下划线分隔的部分
    while '_' in current_type:
        # 找到最后一个下划线的位置
        last_underscore_idx = current_type.rfind('_')
        # 截取到最后一个下划线之前的部分
        current_type = current_type[:last_underscore_idx]
        if current_type:  # 确保不为空
            candidates.append(current_type)
    
    return candidates


def load_robot_config(robot_name, config_type="ros2_control", robot_type=""):
    """
    Get robot configuration from ROS2 controller configuration file.
    
    支持渐进名称匹配：当type为'ccs_left_rg75'时，会依次尝试：
    1. ccs_left_rg75.yaml
    2. ccs_left.yaml
    3. ccs.yaml
    4. 默认配置文件
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'arx5', etc.)
        config_type (str): Type of configuration file (default: 'ros2_control')
        robot_type (str): Robot type/variant (e.g., 'x5', 'r5', 'robotiq85', 'ccs_left_rg75', etc.)
        
    Returns:
        tuple: (config_dict, config_path) or (None, None) if failed
        
    Example:
        >>> config, path = load_robot_config('cr5', 'ros2_control', 'x5')
        >>> if config:
        ...     print(f"Loaded config from: {path}")
    """
    # 创建缓存键
    cache_key = f"{robot_name}_{config_type}_{robot_type}"
    
    # 检查缓存
    if cache_key in _config_cache:
        return _config_cache[cache_key]
    
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None, None
        
    try:
        # 确定配置目录
        if config_type == "ros2_control":
            config_dir = os.path.join(robot_pkg_path, "config", "ros2_control")
            default_config_file = "ros2_controllers.yaml"
        else:
            config_dir = os.path.join(robot_pkg_path, "config", config_type)
            default_config_file = f"{config_type}.yaml"
        
        config_path = None
        config_file = None
        
        # 如果提供了robot_type，尝试渐进匹配
        if robot_type and robot_type.strip():
            type_candidates = _generate_progressive_type_candidates(robot_type)
            
            # 依次尝试每个候选type
            for candidate_type in type_candidates:
                candidate_file = f"{candidate_type}.yaml"
                candidate_path = os.path.join(config_dir, candidate_file)
                
                if os.path.exists(candidate_path):
                    config_path = candidate_path
                    config_file = candidate_file
                    print(f"[INFO] Using {config_type} config file (progressive match): {config_file}")
                    break
            
            # 如果渐进匹配都没有找到，使用默认配置
            if config_path is None:
                config_file = default_config_file
                config_path = os.path.join(config_dir, config_file)
                print(f"[INFO] Progressive type matching failed, using default: {config_file}")
        else:
            # 没有提供robot_type，直接使用默认配置
            config_file = default_config_file
            config_path = os.path.join(config_dir, config_file)
            
        print(f"[INFO] Reading {config_type} config from: {config_path}")
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # 缓存结果
        result = (config, config_path)
        _config_cache[cache_key] = result
            
        return result
        
    except FileNotFoundError:
        print(f"[WARN] {config_type} config file not found for robot '{robot_name}'")
        return None, None
    except yaml.YAMLError as e:
        print(f"[ERROR] Failed to parse YAML config for robot '{robot_name}': {e}")
        return None, None
    except Exception as e:
        print(f"[ERROR] Unexpected error reading config for robot '{robot_name}': {e}")
        return None, None


def get_planning_urdf_path(robot_name, robot_type=""):
    """
    Get planning URDF file path based on robot type.
    
    Args:
        robot_name (str): Name of the robot
        robot_type (str): Robot type/variant
        
    Returns:
        str: Path to planning URDF file, or None if not found
        
    Example:
        >>> urdf_path = get_planning_urdf_path('cr5', 'x5')
        >>> print(urdf_path)
        '/opt/ros/humble/share/cr5_description/urdf/cr5_x5.urdf'
    """
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None
    
    if robot_type and robot_type.strip():
        # Try type-specific URDF first
        robot_identifier = robot_name + "_" + robot_type
        type_specific_urdf = os.path.join(robot_pkg_path, "urdf", robot_identifier + ".urdf")
        
        # Check if type-specific URDF exists
        if os.path.exists(type_specific_urdf):
            return type_specific_urdf
        else:
            # Fallback to default URDF if type-specific doesn't exist
            default_urdf = os.path.join(robot_pkg_path, "urdf", robot_name + ".urdf")
            return default_urdf
    else:
        # Use default URDF
        default_urdf = os.path.join(robot_pkg_path, "urdf", robot_name + ".urdf")
        return default_urdf


def get_info_file_name(robot_name, robot_type="", config_type="ros2_control"):
    """
    Get info_file_name from ROS2 controller configuration, fallback to 'task'.
    
    Args:
        robot_name (str): Name of the robot
        robot_type (str): Robot type/variant
        config_type (str): Type of configuration file
        
    Returns:
        str: Info file name (default: 'task')
        
    Example:
        >>> info_file = get_info_file_name('cr5', 'x5')
        >>> print(info_file)
        'task'
    """
    config, _ = load_robot_config(robot_name, config_type, robot_type)
    
    if config is None:
        return 'task'
    
    try:
        # Extract info_file_name from ocs2_arm_controller parameters
        info_file_name = config.get('ocs2_arm_controller', {}).get('ros__parameters', {}).get('info_file_name', 'task')
        return info_file_name
    except KeyError as e:
        print(f"[WARN] Key error in config for robot '{robot_name}': {e}, using default 'task'")
        return 'task'


def get_gz_bridge_config_path(robot_name):
    """
    Get Gazebo bridge configuration file path for a robot.
    
    首先尝试加载机器人特定的配置，如果不存在则返回默认配置。
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'agibot_g1', etc.)
        
    Returns:
        str: Path to gz_bridge.yaml file (robot-specific or default)
        
    Example:
        >>> bridge_config = get_gz_bridge_config_path('agibot_g1')
        >>> print(f"Using bridge config: {bridge_config}")
    """
    robot_pkg_path = get_robot_package_path(robot_name)
    
    # 首先尝试机器人特定的配置
    if robot_pkg_path is not None:
        gz_bridge_config_path = os.path.join(robot_pkg_path, "config", "gazebo", "gz_bridge.yaml")
        
        if os.path.exists(gz_bridge_config_path):
            print(f"[INFO] Using robot-specific Gazebo bridge config: {gz_bridge_config_path}")
            return gz_bridge_config_path
    
    # 如果没有找到机器人特定的配置，使用默认配置
    try:
        robot_common_launch_path = get_package_share_directory('robot_common_launch')
        default_gz_bridge_config_path = os.path.join(robot_common_launch_path, "config", "gazebo", "gz_bridge.yaml")
        
        if os.path.exists(default_gz_bridge_config_path):
            print(f"[INFO] Using default Gazebo bridge config for robot '{robot_name}': {default_gz_bridge_config_path}")
            return default_gz_bridge_config_path
        else:
            print(f"[WARN] Default Gazebo bridge config not found: {default_gz_bridge_config_path}")
            return None
    except Exception as e:
        print(f"[ERROR] Failed to get default Gazebo bridge config: {e}")
        return None


def get_ros2_control_robot_description(robot_name, robot_type="", hardware="mock_components"):
    """
    生成 ros2_control 的 robot_description。
    
    这个函数用于生成包含 ros2_control 接口的机器人描述，可以在多个地方复用，
    避免重复生成相同的 robot_description。使用缓存机制，相同参数的调用会返回缓存的结果。
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'arx5', etc.)
        robot_type (str): Robot type/variant (e.g., 'x5', 'r5', etc.)
        hardware (str): Hardware type (e.g., 'gz', 'mock_components', 'real', etc.)
        
    Returns:
        str: URDF/XML 格式的机器人描述字符串，如果失败则返回 None
        
    Example:
        >>> robot_description = get_ros2_control_robot_description('cr5', 'x5', 'mock_components')
        >>> if robot_description:
        ...     print("Robot description generated successfully")
    """
    global _robot_description_cache
    
    # 创建缓存键
    cache_key = f"{robot_name}_{robot_type}_{hardware}"
    
    # 检查缓存
    if cache_key in _robot_description_cache:
        return _robot_description_cache[cache_key]
    
    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None
    
    try:
        robot_description_file_path = os.path.join(
            robot_pkg_path,
            "xacro",
            "ros2_control",
            "robot.xacro"
        )
        
        if not os.path.exists(robot_description_file_path):
            print(f"[WARN] ros2_control xacro file not found: {robot_description_file_path}")
            return None
        
        # 构建 xacro mappings
        mappings = {
            'ros2_control_hardware_type': hardware,
        }
        if robot_type and robot_type.strip():
            mappings["type"] = robot_type
        
        # 如果是 Gazebo 模式，添加 gazebo 映射
        if hardware == 'gz':
            mappings['gazebo'] = 'true'
        
        robot_description_config = xacro.process_file(
            robot_description_file_path,
            mappings=mappings
        )
        
        robot_description = robot_description_config.toxml()
        
        # 缓存结果
        _robot_description_cache[cache_key] = robot_description
        
        return robot_description
        
    except Exception as e:
        print(f"[ERROR] Failed to generate ros2_control robot_description for '{robot_name}': {e}")
        return None


def get_gz_image_bridge_topics(robot_name):
    """
    Get Gazebo image bridge topic list for a robot.
    
    仅查找机器人特定的配置，如果不存在则返回None（不启动image bridge）。
    Image bridge是可选的，只有需要相机图像传输的机器人才需要配置。
    
    配置文件格式应该是一个YAML文件，包含一个话题列表：
    ---
    topics:
      - /camera/image
      - /camera/depth_image
    
    Args:
        robot_name (str): Name of the robot (e.g., 'cr5', 'agibot_g1', etc.)
        
    Returns:
        list or None: List of image topics if config exists, otherwise None
        
    Example:
        >>> topics = get_gz_image_bridge_topics('agibot_g1')
        >>> if topics:
        ...     print(f"Image topics: {topics}")
        ... else:
        ...     print("No image bridge needed for this robot")
    """
    robot_pkg_path = get_robot_package_path(robot_name)
    
    if robot_pkg_path is None:
        return None
    
    gz_image_bridge_config_path = os.path.join(robot_pkg_path, "config", "gazebo", "gz_image_bridge.yaml")
    
    if os.path.exists(gz_image_bridge_config_path):
        try:
            with open(gz_image_bridge_config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            if config and 'topics' in config:
                topics = config['topics']
                print(f"[INFO] Found Gazebo image bridge config for robot '{robot_name}' with {len(topics)} topics")
                return topics
            else:
                print(f"[WARN] Image bridge config exists but 'topics' key not found for robot '{robot_name}'")
                return None
        except Exception as e:
            print(f"[ERROR] Failed to read image bridge config for robot '{robot_name}': {e}")
            return None
    else:
        print(f"[INFO] No image bridge config found for robot '{robot_name}', skipping image bridge")
        return None


def parse_task_info(task_file_path):
    """
    解析 task.info 文件，提取 dual_arm_mode 和 control_base_frame 信息。
    
    Args:
        task_file_path (str): task.info 文件路径
        
    Returns:
        tuple: (dual_arm_mode, control_base_frame)
            - dual_arm_mode (bool): 是否为双臂模式
            - control_base_frame (str): 控制基坐标系ID
            
    Example:
        >>> dual_arm, frame = parse_task_info('/path/to/task.info')
        >>> print(f"Dual arm: {dual_arm}, Frame: {frame}")
    """
    dual_arm_mode = False
    control_base_frame = "world"
    
    if not os.path.exists(task_file_path):
        print(f"[WARN] Task file not found: {task_file_path}")
        return dual_arm_mode, control_base_frame
    
    try:
        with open(task_file_path, 'r') as file:
            content = file.read()
        
        # 检查是否有 dualArmMode 配置
        dual_arm_match = re.search(r'dualArmMode\s+true', content)
        if dual_arm_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from task file")
        
        # 检查是否有 eeFrame1（双臂机器人的第二个末端执行器）
        ee_frame1_match = re.search(r'eeFrame1\s+"([^"]+)"', content)
        if ee_frame1_match:
            dual_arm_mode = True
            print(f"[INFO] Detected dual arm mode from eeFrame1: {ee_frame1_match.group(1)}")
        
        # 提取 baseFrame
        base_frame_match = re.search(r'baseFrame\s+"([^"]+)"', content)
        if base_frame_match:
            control_base_frame = base_frame_match.group(1)
            print(f"[INFO] Detected base frame: {control_base_frame}")
        
        print(f"[INFO] Parsed task file - dual_arm_mode: {dual_arm_mode}, control_base_frame: {control_base_frame}")
        
    except Exception as e:
        print(f"[ERROR] Failed to parse task file {task_file_path}: {e}")
    
    return dual_arm_mode, control_base_frame


def prepare_arms_target_manager_parameters(
    task_file_path,
    config_file_path=None,
    marker_fixed_frame=None,
    hand_controllers=None
):
    """
    准备 ArmsTargetManager 节点的参数。
    
    这个函数处理所有参数准备逻辑，包括：
    - 解析 task.info 文件
    - 查找配置文件路径
    - 构建参数字典
    
    Args:
        task_file_path (str): task.info 文件路径
        config_file_path (str, optional): 显式指定的配置文件路径
        marker_fixed_frame (str, optional): marker 固定坐标系。如果为 None，则使用配置文件中的值或节点默认值 'base_link'
        hand_controllers (list, optional): 手部/夹爪控制器名称列表
        
    Note:
        enable_head_control is now configured via YAML config file, not via function parameter
        
    Returns:
        list: 节点参数列表，可以直接传递给 Node 的 parameters 参数
        
    Example:
        >>> params = prepare_arms_target_manager_parameters(
        ...     task_file_path='/path/to/task.info',
        ...     hand_controllers=['left_hand_controller', 'right_hand_controller']
        ... )
        >>> node = Node(package='arms_target_manager', executable='arms_target_manager_node', parameters=params)
    """
    if not task_file_path:
        print(f"[ERROR] No task_file provided")
        return []
    
    # 解析 task.info 文件
    dual_arm_mode, control_base_frame = parse_task_info(task_file_path)
    
    # 确定配置文件路径（优先级：显式指定的config_file > task_file同目录 > 默认配置）
    config_path = None
    
    # 如果显式指定了配置文件路径，优先使用
    if config_file_path:
        if os.path.exists(config_file_path):
            config_path = config_file_path
            print(f"[INFO] Using explicitly specified config file: {config_path}")
        else:
            print(f"[WARN] Specified config file not found: {config_file_path}, falling back to default search")
    
    # 如果还没有找到，检查 task_file 同目录下是否有 target_manager.yaml
    if not config_path:
        task_file_dir = os.path.dirname(task_file_path)
        task_dir_config = os.path.join(task_file_dir, 'target_manager.yaml')
        if os.path.exists(task_dir_config):
            config_path = task_dir_config
            print(f"[INFO] Using config file from task file directory: {config_path}")
    
    # 如果还没有找到，使用默认配置文件
    if not config_path:
        try:
            package_dir = get_package_share_directory('arms_target_manager')
            default_config = os.path.join(package_dir, 'config', 'default.yaml')
            if os.path.exists(default_config):
                config_path = default_config
                print(f"[INFO] Using default config file: {config_path}")
            else:
                print(f"[WARN] Default config file not found: {default_config}, using default parameters")
        except Exception as e:
            print(f"[WARN] Failed to get default config file: {e}")
    
    # 构建参数列表：先加载 YAML 配置，然后用 launch 参数覆盖
    parameters = []
    if config_path:
        parameters.append(config_path)
    
    # 必须覆盖的参数（从 task_file 解析或必需参数）
    override_params = {
        'dual_arm_mode': dual_arm_mode,
        'control_base_frame': control_base_frame
    }
    
    # 只有当明确提供 marker_fixed_frame 时才覆盖配置文件中的值
    if marker_fixed_frame is not None:
        override_params['marker_fixed_frame'] = marker_fixed_frame
    
    # 添加 hand_controllers 参数（如果提供）
    if hand_controllers:
        override_params['hand_controllers'] = hand_controllers
    
    parameters.append(override_params)
    
    return parameters
