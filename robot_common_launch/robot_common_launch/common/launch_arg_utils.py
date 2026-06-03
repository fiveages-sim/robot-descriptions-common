"""
Launch argument utilities: prefixed xacro_/hardware_/control_ routing and robot profile.
"""

from __future__ import annotations

import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml

XACRO_PREFIX = "xacro_"
HARDWARE_PREFIX = "hardware_"
CONTROL_PREFIX = "control_"

REAL_HARDWARE = frozenset({"real", "real_usb"})

CORE_LAUNCH_KEYS = frozenset({
    "robot",
    "type",
    "hardware",
    "use_sim_time",
    "world",
    "world_package",
    "remappings",
    "ctrl_mode",
    "robot_profile",
    "launch_mode",
    "enable_gripper",
    "enable_body",
    "enable_arms_target_manager",
    "planning_use_base_urdf",
})


def _strip_prefix(key: str, prefix: str) -> Optional[str]:
    if key.startswith(prefix):
        return key[len(prefix):]
    return None


def extract_prefixed_args(
    launch_configurations: Dict[str, str],
    prefix: str,
    argv: Optional[List[str]] = None,
) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for key, value in launch_configurations.items():
        if value is None:
            continue
        stripped = _strip_prefix(key, prefix)
        if stripped is not None and str(value).strip():
            result[stripped] = str(value).strip()

    if argv is None:
        argv = sys.argv
    for token in argv:
        if ":=" not in token:
            continue
        key, _, raw_value = token.partition(":=")
        stripped = _strip_prefix(key, prefix)
        if stripped is not None and raw_value.strip():
            result[stripped] = raw_value.strip()
    return result


_PLATFORM_XACRO_KEYS = ("chassis", "variant", "chassis_joints_movable")


def normalize_robot_profile(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Normalize robot profile to {xacro, hardware, control} for launch code.

    New schema (recommended):
      platform:  chassis / variant / arm_ctrl_mode — always apply (incl. quick_start [模板])
      defaults.end_effectors: default EEF for「本机配置」only; overridden by type:= / xacro_* templates
      control.patch: inline ros2_control overrides (deep-merged after compose)
    """
    if not isinstance(data, dict) or not data:
        return {}

    platform = data.get("platform")
    defaults = data.get("defaults")
    if not isinstance(platform, dict) and not isinstance(defaults, dict):
        return data

    legacy_x = data.get("xacro") if isinstance(data.get("xacro"), dict) else {}
    legacy_hw = data.get("hardware") if isinstance(data.get("hardware"), dict) else {}
    legacy_c = data.get("control") if isinstance(data.get("control"), dict) else {}

    xacro: Dict[str, Any] = {}
    hardware = dict(legacy_hw)
    control: Dict[str, Any] = {}

    plat = platform if isinstance(platform, dict) else {}
    for key in _PLATFORM_XACRO_KEYS:
        if key in plat:
            xacro[key] = plat[key]
        elif key in legacy_x:
            xacro[key] = legacy_x[key]

    arm_mode = plat.get("arm_ctrl_mode") or legacy_hw.get("arm_ctrl_mode")
    if arm_mode is not None and str(arm_mode).strip():
        hardware["arm_ctrl_mode"] = str(arm_mode).strip()

    if isinstance(defaults, dict):
        raw_eef = defaults.get("end_effectors")
        eef = raw_eef if isinstance(raw_eef, dict) else defaults
        if isinstance(eef, dict):
            sym = str(eef.get("type", "") or "").strip()
            left = str(eef.get("left", "") or eef.get("left_type", "") or "").strip()
            right = str(eef.get("right", "") or eef.get("right_type", "") or "").strip()
            if sym:
                xacro["type"] = sym
            if left:
                xacro["left_type"] = left
                control["left"] = left
            if right:
                xacro["right_type"] = right
                control["right"] = right
    else:
        for key in ("type", "left_type", "right_type"):
            if legacy_x.get(key) is not None:
                xacro[key] = legacy_x[key]
        for key in ("left", "right"):
            if legacy_c.get(key):
                control[key] = legacy_c[key]

    patch = legacy_c.get("patch")
    if isinstance(patch, dict) and patch:
        control["patch"] = patch

    return {"xacro": xacro, "hardware": hardware, "control": control}


def load_robot_profile(profile_path: str) -> Dict[str, Any]:
    if not profile_path or not os.path.isfile(profile_path):
        return {}
    with open(profile_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return {}
    return normalize_robot_profile(data)


def resolve_profile_path(launch_configurations: Dict[str, str]) -> str:
    profile = launch_configurations.get("robot_profile", "").strip()
    if profile and os.path.isfile(profile):
        return os.path.abspath(profile)
    return ""


def resolve_control_patch(profile: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """ros2_control overrides from robot.local.yaml control.patch (deep-merged after compose)."""
    if not profile:
        return {}
    control_section = profile.get("control") or {}
    if not isinstance(control_section, dict):
        return {}
    patch = control_section.get("patch")
    return dict(patch) if isinstance(patch, dict) else {}


def resolve_control_sides(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    control_args = extract_prefixed_args(launch_configurations, CONTROL_PREFIX)
    left = control_args.get("left", "")
    right = control_args.get("right", "")
    launch_type = launch_configurations.get("type", "").strip()
    # Symmetric menu template (type:= only): override profile left/right for ros2_control compose.
    if launch_type and not left and not right:
        return launch_type, launch_type
    if profile:
        control_section = profile.get("control") or {}
        if isinstance(control_section, dict):
            if not left:
                left = str(control_section.get("left", "") or "")
            if not right:
                right = str(control_section.get("right", "") or "")
        xacro_section = profile.get("xacro") or {}
        if isinstance(xacro_section, dict):
            xl = str(xacro_section.get("left_type", "") or "")
            xr = str(xacro_section.get("right_type", "") or "")
            xt = str(xacro_section.get("type", "") or "")
            if not left:
                left = xl or xt
            if not right:
                right = xr or xt
    return left.strip(), right.strip()


def is_asymmetric_eef(left_type: str, right_type: str, launch_type: str) -> bool:
    if left_type and right_type and left_type != right_type:
        return True
    if left_type or right_type:
        other = launch_type.strip()
        if left_type and other and left_type != other:
            return True
        if right_type and other and right_type != other:
            return True
    return False


def should_use_base_planning_urdf(
    launch_configurations: Dict[str, str],
    left_type: str,
    right_type: str,
) -> bool:
    flag = launch_configurations.get("planning_use_base_urdf", "").strip().lower()
    if flag in ("true", "1", "yes"):
        return True
    launch_type = launch_configurations.get("type", "").strip()
    return is_asymmetric_eef(left_type, right_type, launch_type)


def build_xacro_mappings(
    hardware: str,
    launch_configurations: Dict[str, str],
    robot_profile: Optional[str] = None,
) -> Dict[str, str]:
    profile = load_robot_profile(robot_profile) if robot_profile else {}
    profile_xacro = profile.get("xacro") or {}
    if not isinstance(profile_xacro, dict):
        profile_xacro = {}

    mappings: Dict[str, str] = {"ros2_control_hardware_type": hardware}

    for key, value in profile_xacro.items():
        if value is not None and str(value).strip():
            mappings[str(key)] = str(value).strip()

    xacro_overrides = extract_prefixed_args(launch_configurations, XACRO_PREFIX)
    mappings.update(xacro_overrides)

    if hardware in REAL_HARDWARE:
        profile_hardware = profile.get("hardware") or {}
        if isinstance(profile_hardware, dict):
            for key, value in profile_hardware.items():
                if value is not None and str(value).strip():
                    mappings[str(key)] = str(value).strip()
        mappings.update(extract_prefixed_args(launch_configurations, HARDWARE_PREFIX))

    launch_type = launch_configurations.get("type", "").strip()
    explicit_left = str(xacro_overrides.get("left_type", "") or "").strip()
    explicit_right = str(xacro_overrides.get("right_type", "") or "").strip()

    if launch_type:
        mappings["type"] = launch_type
    elif "type" not in mappings:
        xt = str(profile_xacro.get("type", "") or "").strip()
        if xt:
            mappings["type"] = xt

    if explicit_left:
        mappings["left_type"] = explicit_left
    elif launch_type and not explicit_left and not explicit_right:
        mappings.pop("left_type", None)
    else:
        left_type = str(profile_xacro.get("left_type", "") or "").strip()
        if left_type:
            mappings["left_type"] = left_type

    if explicit_right:
        mappings["right_type"] = explicit_right
    elif launch_type and not explicit_left and not explicit_right:
        mappings.pop("right_type", None)
    else:
        right_type = str(profile_xacro.get("right_type", "") or "").strip()
        if right_type:
            mappings["right_type"] = right_type

    if hardware == "gz":
        mappings["gazebo"] = "true"

    if "collider" not in mappings:
        mappings["collider"] = "simple"

    mount_robot = (
        launch_configurations.get("robot") or launch_configurations.get("robot_name") or ""
    ).strip()
    if mount_robot:
        from .robot_utils import apply_host_arm_mount_config

        apply_host_arm_mount_config(
            mappings,
            mount_robot,
            str(mappings.get("type", "") or "").strip(),
        )

    return mappings


def forward_robot_launch_args(context, extra_core_keys: Optional[List[str]] = None) -> List[Tuple[str, str]]:
    configs = context.launch_configurations
    core = set(CORE_LAUNCH_KEYS)
    if extra_core_keys:
        core.update(extra_core_keys)

    args: List[Tuple[str, str]] = []
    seen = set()
    for key, value in configs.items():
        if value is None:
            continue
        if key in core or key.startswith((XACRO_PREFIX, HARDWARE_PREFIX, CONTROL_PREFIX)):
            args.append((key, str(value)))
            seen.add(key)

    for prefix in (XACRO_PREFIX, HARDWARE_PREFIX, CONTROL_PREFIX):
        for name, value in extract_prefixed_args(configs, prefix, argv=sys.argv).items():
            full_key = f"{prefix}{name}"
            if full_key not in seen:
                args.append((full_key, value))
                seen.add(full_key)
    return args


def create_robot_profile_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "robot_profile",
            default_value="",
            description="Path to robot.local.yaml (machine hardware profile)",
        ),
        DeclareLaunchArgument(
            "planning_use_base_urdf",
            default_value="",
            description="If true, OCS2 planning uses base {robot}.urdf without type suffix",
        ),
    ]
