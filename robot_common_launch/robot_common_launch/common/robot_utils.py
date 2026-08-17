"""
Common robot utilities for launch files.

This module provides utility functions for robot path management and configuration loading.
"""

import hashlib
import json
import os
import re
import tempfile
from dataclasses import dataclass

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

from .control_compose import (
    compose_control_config,
    is_compose_asymmetric,
    resolve_compose_type_key,
)
from .launch_arg_utils import (
    build_xacro_mappings,
    resolve_profile_path,
    resolve_robot_arms,
)

# 全局缓存字典，避免重复读取配置文件
_config_cache = {}

# 全局缓存字典，避免重复生成 robot_description
_robot_description_cache = {}


@dataclass(frozen=True)
class RobotConfigMeta:
    """Metadata from load_robot_config; single source for merged-file decisions."""

    compose_applied: bool
    type_yaml_found: bool
    patch_applied: bool
    variant_overlay_applied: bool
    needs_merged_file: bool
    merged_yaml_path: str
    base_config_path: str = ""
    hardware_overlay_applied: bool = False


EMPTY_ROBOT_CONFIG_META = RobotConfigMeta(
    compose_applied=False,
    type_yaml_found=False,
    patch_applied=False,
    variant_overlay_applied=False,
    needs_merged_file=False,
    merged_yaml_path="",
    base_config_path="",
    hardware_overlay_applied=False,
)


def _build_robot_config_meta(
    *,
    compose_applied: bool,
    type_yaml_found: bool,
    patch_applied: bool,
    variant_overlay_applied: bool,
    hardware_overlay_applied: bool,
    config: dict,
    config_path: str,
    yaml_only: bool,
) -> RobotConfigMeta:
    if yaml_only:
        return EMPTY_ROBOT_CONFIG_META
    needs_merged = (
        compose_applied
        or patch_applied
        or variant_overlay_applied
        or hardware_overlay_applied
    )
    merged_path = ""
    if needs_merged and config:
        merged_path = write_temp_ros2_control_yaml(config, quiet=True)
    return RobotConfigMeta(
        compose_applied=compose_applied,
        type_yaml_found=type_yaml_found,
        patch_applied=patch_applied,
        variant_overlay_applied=variant_overlay_applied,
        needs_merged_file=needs_merged,
        merged_yaml_path=merged_path,
        base_config_path=config_path or "",
        hardware_overlay_applied=hardware_overlay_applied,
    )


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


def _deep_merge_dicts(base, override):
    """
    Recursively merge dictionaries without mutating inputs.

    Values from override win when both sides define the same key.
    """
    if not isinstance(base, dict):
        base = {}
    if not isinstance(override, dict):
        return override

    merged = dict(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _deep_merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def write_temp_ros2_control_yaml(config_dict, *, quiet: bool = False):
    """Write merged ros2_control config to a temp YAML file for ros2_control_node."""
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="ros2_control_merged_")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        yaml.safe_dump(config_dict, handle, default_flow_style=False, sort_keys=False)
    if not quiet:
        print(f"[INFO] Wrote merged ros2_control config: {path}")
    return path


def _strip_eef_controllers_from_config(config):
    """Remove per-side gripper/hand sections before EEF compose merge."""
    for controller_name in list(config.keys()):
        if controller_name.endswith("_gripper_controller") or controller_name.endswith(
            "_hand_controller"
        ):
            config.pop(controller_name, None)
        if controller_name == "controller_manager":
            cm_params = config.get("controller_manager", {}).get("ros__parameters", {})
            if isinstance(cm_params, dict):
                for cm_name in list(cm_params.keys()):
                    if cm_name.endswith("_gripper_controller") or cm_name.endswith(
                        "_hand_controller"
                    ):
                        cm_params.pop(cm_name, None)


def load_robot_config(
    robot_name,
    config_type="ros2_control",
    robot_type="",
    control_left="",
    control_right="",
    control_patch=None,
    robot_variant="",
    hardware="",
    yaml_only=False,
):
    """
    Load ros2_control config with optional per-side compose and profile patch merge.

    Merge order: common.yaml + type yaml + variant yaml + hardware yaml
    + compose + control.patch

    When yaml_only is True, skip symmetric registry/template compose fallback (used by
    control_compose enrichment to avoid recursion).
    """
    effective_type, left, right = resolve_compose_type_key(robot_type, control_left, control_right)
    asymmetric = is_compose_asymmetric(left, right)
    patch = control_patch if isinstance(control_patch, dict) else {}
    variant_key = str(robot_variant or "").strip()
    hardware_key = str(hardware or "").strip()
    patch_stamp = (
        hashlib.md5(json.dumps(patch, sort_keys=True).encode()).hexdigest() if patch else ""
    )

    cache_key = (
        f"{robot_name}_{config_type}_{effective_type}_{left}_{right}"
        f"_{variant_key}_{hardware_key}_{patch_stamp}"
    )
    if not yaml_only and cache_key in _config_cache:
        return _config_cache[cache_key]

    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None, None, EMPTY_ROBOT_CONFIG_META

    compose_applied = False
    try:
        if config_type == "ros2_control":
            config_dir = os.path.join(robot_pkg_path, "config", "ros2_control")
            default_config_file = "ros2_controllers.yaml"
        else:
            config_dir = os.path.join(robot_pkg_path, "config", config_type)
            default_config_file = f"{config_type}.yaml"

        config_path = None
        config_file = None
        type_yaml_found = False

        if effective_type and effective_type.strip() and not asymmetric:
            type_candidates = _generate_progressive_type_candidates(effective_type)
            for candidate_type in type_candidates:
                candidate_file = f"{candidate_type}.yaml"
                candidate_path = os.path.join(config_dir, candidate_file)
                if os.path.exists(candidate_path):
                    config_path = candidate_path
                    config_file = candidate_file
                    type_yaml_found = True
                    print(f"[INFO] Using {config_type} config file (progressive match): {config_file}")
                    break
            if config_path is None:
                config_file = default_config_file
                config_path = os.path.join(config_dir, config_file)
                print(f"[INFO] Progressive type matching failed, using default: {config_file}")
        else:
            config_file = default_config_file
            config_path = os.path.join(config_dir, config_file)
            if asymmetric:
                print(f"[INFO] Asymmetric EEF compose: {left} + {right}, base config: {config_file}")

        common_config_path = os.path.join(config_dir, "common.yaml")
        common_config = {}
        if config_type == "ros2_control" and os.path.exists(common_config_path):
            with open(common_config_path, "r") as file:
                common_config = yaml.safe_load(file) or {}

        if common_config:
            print(
                f"[INFO] Loaded {config_type} config: {config_path} "
                f"(+ {common_config_path})"
            )
        else:
            print(f"[INFO] Loaded {config_type} config: {config_path}")

        with open(config_path, "r") as file:
            config = yaml.safe_load(file) or {}

        config = _deep_merge_dicts(common_config, config)

        variant_overlay_applied = False
        if variant_key:
            variant_config_path = os.path.join(config_dir, f"{variant_key}.yaml")
            if os.path.isfile(variant_config_path):
                with open(variant_config_path, "r") as file:
                    variant_config = yaml.safe_load(file) or {}
                config = _deep_merge_dicts(config, variant_config)
                variant_overlay_applied = True
                print(
                    f"[INFO] Merged {config_type} variant overlay: "
                    f"{os.path.basename(variant_config_path)}"
                )

        hardware_overlay_applied = False
        if hardware_key:
            hardware_config_path = os.path.join(config_dir, f"{hardware_key}.yaml")
            if os.path.isfile(hardware_config_path):
                with open(hardware_config_path, "r") as file:
                    hardware_config = yaml.safe_load(file) or {}
                config = _deep_merge_dicts(config, hardware_config)
                hardware_overlay_applied = True
                print(
                    f"[INFO] Merged {config_type} hardware overlay: "
                    f"{os.path.basename(hardware_config_path)}"
                )

        use_compose = asymmetric or (
            not yaml_only
            and not type_yaml_found
            and bool(effective_type and effective_type.strip())
        )
        if use_compose:
            _strip_eef_controllers_from_config(config)

        if use_compose:
            composed = compose_control_config(left, right, robot_name, base_config=config)
            if composed:
                config = _deep_merge_dicts(config, composed)
                compose_applied = True
                if asymmetric:
                    print(f"[INFO] Merged composed control config for {left} + {right}")
                else:
                    print(
                        f"[INFO] Merged symmetric EEF compose (template fallback) "
                        f"for {effective_type}"
                    )

        patch_applied = bool(patch)
        if patch_applied:
            config = _deep_merge_dicts(config, patch)
            print("[INFO] Merged control.patch from robot profile")

        meta = _build_robot_config_meta(
            compose_applied=compose_applied,
            type_yaml_found=type_yaml_found,
            patch_applied=patch_applied,
            variant_overlay_applied=variant_overlay_applied,
            hardware_overlay_applied=hardware_overlay_applied,
            config=config,
            config_path=config_path or "",
            yaml_only=yaml_only,
        )

        result = (config, config_path, meta)
        if not yaml_only:
            _config_cache[cache_key] = result
        return result

    except FileNotFoundError:
        print(f"[WARN] {config_type} config file not found for robot '{robot_name}'")
        return None, None, EMPTY_ROBOT_CONFIG_META
    except yaml.YAMLError as e:
        print(f"[ERROR] Failed to parse YAML config for robot '{robot_name}': {e}")
        return None, None, EMPTY_ROBOT_CONFIG_META
    except Exception as e:
        print(f"[ERROR] Unexpected error reading config for robot '{robot_name}': {e}")
        return None, None, EMPTY_ROBOT_CONFIG_META


def extract_info_file_name_from_config(config, launch_mode=None, default="task"):
    """
    Read OCS2 task .info stem from merged ros2_control config.

    launch_mode: full_body → wbc; split_body / demo → arm; None → wbc then arm.
    """
    if not isinstance(config, dict):
        return default

    mode = (launch_mode or "").strip()
    if mode == "full_body":
        order = ("ocs2_wbc_controller", "ocs2_arm_controller")
    elif mode in ("split_body", "demo"):
        # Do not fall back to ocs2_wbc (e.g. fixed_base on humanoid); planning uses arm .info only.
        order = ("ocs2_arm_controller",)
    else:
        order = ("ocs2_wbc_controller", "ocs2_arm_controller")

    for controller_key in order:
        params = config.get(controller_key, {}).get("ros__parameters", {})
        if not isinstance(params, dict):
            continue
        name = str(params.get("info_file_name", "") or "").strip()
        if name:
            return name.removesuffix(".info")
    return default


def get_info_file_name(
    robot_name,
    robot_type="",
    config_type="ros2_control",
    control_left="",
    control_right="",
    control_patch=None,
):
    """
    Get info_file_name from merged ROS2 controller configuration, fallback to 'task'.

    Uses the same merge order as load_robot_config (compose + control.patch).
    """
    config, _, _meta = load_robot_config(
        robot_name,
        config_type,
        robot_type,
        control_left=control_left,
        control_right=control_right,
        control_patch=control_patch,
    )
    return extract_info_file_name_from_config(config, launch_mode=None)


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


def _mappings_cache_key(robot_name, hardware, mappings):
    items = sorted((str(k), str(v)) for k, v in mappings.items())
    return f"{robot_name}_{hardware}_{items}"


def get_ros2_control_robot_description(
    robot_name,
    robot_type="",
    hardware="mock_components",
    mappings=None,
    launch_configurations=None,
    robot_profile=None,
):
    """
    Generate ros2_control robot_description from xacro with optional mappings dict
    or launch profile / prefixed arguments.
    """
    global _robot_description_cache

    if mappings is None:
        if launch_configurations is not None:
            profile_path = robot_profile or resolve_profile_path(launch_configurations)
            mappings = build_xacro_mappings(hardware, launch_configurations, profile_path or None)
        else:
            mappings = {"ros2_control_hardware_type": hardware}
            if robot_type and robot_type.strip():
                mappings["type"] = robot_type
            if hardware == "gz":
                mappings["gazebo"] = "true"
    # ros2_control: EEF stays actuated for adaptive_gripper_controller; OCS2 uses planning URDF only.
    mappings["eef_fixed_joints"] = "false"

    cache_key = _mappings_cache_key(robot_name, hardware, mappings)
    if cache_key in _robot_description_cache:
        return _robot_description_cache[cache_key]

    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        return None

    try:
        robot_description_file_path = os.path.join(
            robot_pkg_path, "xacro", "ros2_control", "robot.xacro"
        )
        if not os.path.exists(robot_description_file_path):
            print(f"[WARN] ros2_control xacro file not found: {robot_description_file_path}")
            return None

        robot_description_config = xacro.process_file(
            robot_description_file_path,
            mappings=mappings,
        )
        robot_description = robot_description_config.toxml()
        _robot_description_cache[cache_key] = robot_description
        return robot_description

    except Exception as e:
        print(f"[ERROR] Failed to generate ros2_control robot_description for '{robot_name}': {e}")
        return None


def get_planning_urdf_cache_dir(robot_name):
    """Writable cache directory for xacro-generated planning URDF files.

    Defaults to /tmp so reboot clears stale xacro caches. Override with
    OCS2_PLANNING_URDF_DIR when a persistent cache is desired.
    """
    if override := os.environ.get("OCS2_PLANNING_URDF_DIR", "").strip():
        base = override
    else:
        base = "/tmp"
    cache_dir = os.path.join(base, "ocs2_ros2", robot_name, "planning_urdf")
    os.makedirs(cache_dir, exist_ok=True)
    return cache_dir


def write_planning_urdf_cache(robot_name, urdf_content, cache_key=""):
    """Write planning URDF XML to cache; returns absolute file path."""
    if not urdf_content:
        return ""
    suffix = cache_key if cache_key else "default"
    cache_path = os.path.join(get_planning_urdf_cache_dir(robot_name), f"{suffix}.urdf")
    with open(cache_path, "w", encoding="utf-8") as handle:
        handle.write(urdf_content)
    print(f"[INFO] Wrote planning URDF cache: {cache_path}")
    return os.path.abspath(cache_path)


PLANNING_SCOPE_ARMS = "arms"
PLANNING_SCOPE_FULL = "full"

_XACRO_SIDE_EEF_SUPPORT_CACHE: dict[str, bool] = {}
_XACRO_SIDE_EEF_ARG_PATTERN = re.compile(
    r'<xacro:arg\s+name=["\'](?P<name>left_type|right_type)["\']'
)


def _planning_xacro_supports_side_eef(robot_name: str) -> bool:
    """True when planning xacro declares per-side EEF args (left_type/right_type)."""
    if robot_name in _XACRO_SIDE_EEF_SUPPORT_CACHE:
        return _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name]

    robot_pkg_path = get_robot_package_path(robot_name)
    if robot_pkg_path is None:
        _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = False
        return False

    planning_xacro = os.path.join(robot_pkg_path, "xacro", "robot.xacro")
    if not os.path.isfile(planning_xacro):
        _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = False
        return False

    try:
        with open(planning_xacro, "r", encoding="utf-8") as xacro_file:
            declared = {
                match.group("name")
                for match in _XACRO_SIDE_EEF_ARG_PATTERN.finditer(xacro_file.read())
            }
    except OSError:
        _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = False
        return False

    supported = declared.issuperset({"left_type", "right_type"})
    _XACRO_SIDE_EEF_SUPPORT_CACHE[robot_name] = supported
    return supported


# Public alias for control_compose single/dual-arm topology detection.
planning_xacro_supports_side_eef = _planning_xacro_supports_side_eef


def _resolve_planning_scope(planning_robot_name, launch_configurations, planning_scope=""):
    """Resolve planning URDF scope from explicit args only.

    Prefer the ``planning_scope`` parameter (from launch helpers) or the
    ``planning_scope`` launch configuration. Robot names are not special-cased —
    split/demo/full_body launches already pass ``arms`` or ``full``.
    Unspecified scope defaults to ``full``.

    ``planning_robot_name`` is kept for call-site compatibility only.
    """
    configs = launch_configurations or {}
    scope = (planning_scope or configs.get("planning_scope", "")).strip().lower()
    if scope in (PLANNING_SCOPE_ARMS, PLANNING_SCOPE_FULL):
        return scope
    return PLANNING_SCOPE_FULL


def _resolve_planning_robot_name(planning_robot_name, planning_scope):
    """Use ocs2_arm_controller.robot_name from config; do not override per scope."""
    return planning_robot_name


def _planning_xacro_mappings(
    hardware,
    launch_configurations,
    robot_profile=None,
    planning_scope=PLANNING_SCOPE_FULL,
    planning_robot_name="",
):
    """Xacro mappings for OCS2 planning URDF (kinematic tree without EEF/chassis actuation DOF)."""
    configs = launch_configurations or {}

    mappings = build_xacro_mappings(hardware, configs, robot_profile)
    mappings.pop("ros2_control_hardware_type", None)
    mappings.pop("gazebo", None)
    # Planning URDF: freeze chassis wheels/steer and EEF actuation in xacro (not via .info removeJoints).
    mappings["eef_fixed_joints"] = "true"
    mappings["chassis_joints_movable"] = "false"

    if planning_scope == PLANNING_SCOPE_ARMS:
        mappings.pop("chassis", None)
        # Same-package dual-arm tree rooted at arm_base (arx_lift split, cobot_magic_v1 demo).
        # Do not override an explicit topology from launch / profile.
        mappings.setdefault("topology", "dual")
        if _planning_xacro_supports_side_eef(planning_robot_name):
            left_type = mappings.get("left_type", "").strip()
            right_type = mappings.get("right_type", "").strip()
            if left_type and right_type:
                mappings.pop("type", None)
            elif not mappings.get("type", "").strip():
                # Let xacro/robot.xacro default apply (m6_ccs: dual, galbot_one: hitbot, …).
                mappings.pop("type", None)
        else:
            # xacro has no left_type/right_type: keep symmetric launch ``type``.
            mappings.pop("left_type", None)
            mappings.pop("right_type", None)
            if not mappings.get("type", "").strip():
                # Do not inject a synthetic type; let xacro/robot.xacro default apply
                # (e.g. galaxea_a1 default a1, cr5 default empty).
                mappings.pop("type", None)

    # Standalone AR5 planning xacro selects CCS generation via ``variant``, using
    # the same keys as the humanoid ``arms`` slot.
    if planning_robot_name in ("ar5_ccs", "ar5_srs"):
        arms_key = str(mappings.get("arms") or "").strip()
        if not arms_key:
            arms_key = resolve_robot_arms(
                configs,
                robot_name=str(configs.get("robot") or ""),
            )
        if arms_key:
            mappings["variant"] = arms_key
        mappings.pop("arms", None)

    return mappings


def _planning_urdf_cache_key(mappings, planning_scope=PLANNING_SCOPE_FULL):
    items = "|".join(f"{k}={v}" for k, v in sorted(mappings.items()))
    items += f"|schema=v13|scope={planning_scope}"
    return hashlib.sha256(items.encode("utf-8")).hexdigest()[:16]


def build_planning_urdf_launch_params(
    planning_robot_name,
    launch_configurations,
    hardware="mock_components",
    robot_profile=None,
    planning_scope="",
):
    """
    Generate planning URDF from xacro/robot.xacro and return controller params.

    planning_scope:
      - arms  : split-body OCS2 — dual-arm tree (injects topology:=dual; root arm_base)
      - full  : full-body WBC — whole robot (e.g. fiveages_w2 / cobot_magic_v1 xacro)

    Returns dict with planning_urdf_variant / planning_urdf_path, or empty dict
    when xacro planning URDF cannot be generated.
    """
    configs = launch_configurations or {}

    profile_path = robot_profile
    if not profile_path:
        profile_path = resolve_profile_path(configs) or None

    scope = _resolve_planning_scope(planning_robot_name, configs, planning_scope)
    effective_robot_name = _resolve_planning_robot_name(planning_robot_name, scope)
    if effective_robot_name != planning_robot_name:
        print(
            f"[INFO] planning_scope={scope}: using '{effective_robot_name}' "
            f"planning URDF (launch/config robot: {planning_robot_name})"
        )

    mappings = _planning_xacro_mappings(
        hardware, configs, profile_path, scope, effective_robot_name
    )

    robot_pkg_path = get_robot_package_path(effective_robot_name)
    if robot_pkg_path is None:
        return {}

    planning_xacro = os.path.join(robot_pkg_path, "xacro", "robot.xacro")
    if not os.path.isfile(planning_xacro):
        print(f"[ERROR] No planning xacro at {planning_xacro}; cannot generate planning URDF")
        return {}

    cache_key = _planning_urdf_cache_key(mappings, scope)
    cache_path = os.path.join(get_planning_urdf_cache_dir(effective_robot_name), f"{cache_key}.urdf")
    if os.path.isfile(cache_path):
        print(f"[INFO] Reusing cached planning URDF: {cache_path}")
        return {
            "planning_urdf_variant": "xacro",
            "planning_urdf_path": cache_path,
        }

    urdf_xml = get_planning_robot_description(
        effective_robot_name,
        launch_configurations=configs,
        robot_profile=profile_path,
        hardware=hardware,
        planning_scope=scope,
    )
    if not urdf_xml:
        print(f"[WARN] Failed to generate planning URDF from xacro for '{effective_robot_name}'")
        return {}

    written_path = write_planning_urdf_cache(effective_robot_name, urdf_xml, cache_key)
    if not written_path:
        return {}

    return {
        "planning_urdf_variant": "xacro",
        "planning_urdf_path": written_path,
    }


def resolve_planning_urdf_file_or_fail(
    robot_name,
    launch_configurations,
    hardware="mock_components",
    robot_profile=None,
    planning_scope="",
):
    """Return cached xacro planning URDF path, or None after logging an error."""
    profile_path = robot_profile
    if not profile_path:
        profile_path = resolve_profile_path(launch_configurations or {}) or None

    params = build_planning_urdf_launch_params(
        robot_name,
        launch_configurations,
        hardware,
        profile_path,
        planning_scope=planning_scope,
    )
    path = (params.get("planning_urdf_path") or "").strip()
    if params.get("planning_urdf_variant") == "xacro" and path and os.path.isfile(path):
        return path

    scope_label = planning_scope or "default"
    print(
        f"[ERROR] Failed to resolve xacro planning URDF for '{robot_name}' "
        f"(scope={scope_label}): variant={params.get('planning_urdf_variant')!r} path={path!r}"
    )
    return None


def get_planning_robot_description(
    robot_name,
    launch_configurations=None,
    robot_profile=None,
    hardware="mock_components",
    planning_scope="",
):
    """Generate kinematic planning URDF from xacro/robot.xacro (no ros2_control)."""
    profile_path = robot_profile
    if not profile_path and launch_configurations is not None:
        profile_path = resolve_profile_path(launch_configurations) or None

    scope = _resolve_planning_scope(robot_name, launch_configurations or {}, planning_scope)
    effective_robot_name = _resolve_planning_robot_name(robot_name, scope)
    mappings = _planning_xacro_mappings(
        hardware,
        launch_configurations or {},
        profile_path,
        scope,
        effective_robot_name,
    )

    robot_pkg_path = get_robot_package_path(effective_robot_name)
    if robot_pkg_path is None:
        return None

    planning_xacro = os.path.join(robot_pkg_path, "xacro", "robot.xacro")
    if not os.path.exists(planning_xacro):
        print(f"[WARN] Planning xacro not found: {planning_xacro}")
        return None

    try:
        return xacro.process_file(planning_xacro, mappings=mappings).toxml()
    except Exception as e:
        print(f"[ERROR] Failed to generate planning robot_description for '{robot_name}': {e}")
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
        tuple: (dual_arm_mode, control_base_frame, marker_fixed_frame)
            - dual_arm_mode (bool): 是否为双臂模式
            - control_base_frame (str): 控制基坐标系ID
            - marker_fixed_frame (str or None): marker固定坐标系，None表示由target_manager.yaml决定

    Example:
        >>> dual_arm, frame, marker_frame = parse_task_info('/path/to/task.info')
        >>> print(f"Dual arm: {dual_arm}, Frame: {frame}, Marker frame: {marker_frame}")
    """
    dual_arm_mode = False
    control_base_frame = "world"
    marker_fixed_frame = None

    if not os.path.exists(task_file_path):
        print(f"[WARN] Task file not found: {task_file_path}")
        return dual_arm_mode, control_base_frame, marker_fixed_frame

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

        # 轮式底盘模式（manipulatorModelType=1）下，OCS2目标和EE均在world坐标系下
        # marker也必须锚定在world，否则底盘移动时marker跟着漂移
        model_type_match = re.search(r'manipulatorModelType\s+(\d+)', content)
        if model_type_match and int(model_type_match.group(1)) == 1:
            control_base_frame = "world"
            marker_fixed_frame = "world"
            print(f"[INFO] Wheel-based mode detected, overriding control_base_frame and marker_fixed_frame to 'world'")

        print(f"[INFO] Parsed task file - dual_arm_mode: {dual_arm_mode}, control_base_frame: {control_base_frame}, marker_fixed_frame: {marker_fixed_frame}")

    except Exception as e:
        print(f"[ERROR] Failed to parse task file {task_file_path}: {e}")

    return dual_arm_mode, control_base_frame, marker_fixed_frame


def prepare_arms_target_manager_parameters(
    task_file_path,
    config_file_path=None,
    marker_fixed_frame=None,
    hand_controllers=None,
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
    dual_arm_mode, control_base_frame, auto_marker_fixed_frame = parse_task_info(task_file_path)

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
    
    # marker_fixed_frame 优先级：函数参数 > task.info自动检测 > target_manager.yaml
    effective_marker_fixed_frame = marker_fixed_frame if marker_fixed_frame is not None else auto_marker_fixed_frame
    if effective_marker_fixed_frame is not None:
        override_params['marker_fixed_frame'] = effective_marker_fixed_frame
    
    # 添加 hand_controllers 参数（如果提供）
    if hand_controllers:
        override_params['hand_controllers'] = hand_controllers

    parameters.append(override_params)
    
    return parameters
