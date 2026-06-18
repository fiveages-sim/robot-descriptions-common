"""Compose ros2_control yaml fragments from per-side end-effector templates."""

from __future__ import annotations

import copy
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

_REGISTRY_CACHE: Optional[Dict[str, Dict[str, str]]] = None
_REGISTRY_DEFAULTS_KEY = "_defaults"
_BUILTIN_GRIPPER_DEFAULT = {
    "package": "robot_common_launch",
    "template": "config/ros2_control/templates/gripper.side.yaml",
    "category": "gripper",
}

_MOTION_CONTROLLER_SECTIONS = (
    "ocs2_wbc_controller",
    "ocs2_arm_controller",
    "body_joint_controller",
    "head_joint_controller",
)


def _is_eef_joint(joint_name: str) -> bool:
    return joint_name.endswith("_gripper_joint") or "_hand_" in joint_name


def _extract_base_joint_state_joints(config: Dict[str, Any]) -> List[str]:
    """Derive non-EEF joints for joint_state_broadcaster from robot package config."""
    jsb_joints = (
        config.get("joint_state_broadcaster", {})
        .get("ros__parameters", {})
        .get("joints", [])
    )
    if jsb_joints:
        filtered = [j for j in jsb_joints if isinstance(j, str) and not _is_eef_joint(j)]
        if filtered:
            return filtered

    joints: List[str] = []
    seen: set[str] = set()

    for section in _MOTION_CONTROLLER_SECTIONS:
        section_joints = (
            config.get(section, {}).get("ros__parameters", {}).get("joints", []) or []
        )
        for joint in section_joints:
            if not isinstance(joint, str) or _is_eef_joint(joint):
                continue
            if joint in seen:
                continue
            joints.append(joint)
            seen.add(joint)

    return joints


def _deep_merge_dicts(base, override):
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


def _load_registry() -> Dict[str, Dict[str, str]]:
    global _REGISTRY_CACHE
    if _REGISTRY_CACHE is not None:
        return _REGISTRY_CACHE
    registry_path = os.path.join(
        get_package_share_directory("robot_common_launch"),
        "config",
        "eef_control_registry.yaml",
    )
    with open(registry_path, "r", encoding="utf-8") as handle:
        _REGISTRY_CACHE = yaml.safe_load(handle) or {}
    return _REGISTRY_CACHE


def _is_registry_eef_entry(entry: Any) -> bool:
    return isinstance(entry, dict) and "template" in entry


_PASSIVE_EEF_TYPES = frozenset({"suction_cup"})


def _is_passive_eef_type(eef_type: str) -> bool:
    """End effectors with no ros2_control joints or controllers."""
    return eef_type.strip().lower() in _PASSIVE_EEF_TYPES


def _is_probable_hand_type(eef_type: str) -> bool:
    """Heuristic: unregistered hand-like keys should not get gripper fallback."""
    normalized = eef_type.strip().lower()
    if not normalized or normalized == "none":
        return False
    if normalized.startswith("linkerhand"):
        return True
    if "hand" in normalized and "gripper" not in normalized:
        return True
    if "suction" in normalized:
        return True
    return False


def _gripper_default_entry(registry: Dict[str, Any]) -> Dict[str, Any]:
    defaults = registry.get(_REGISTRY_DEFAULTS_KEY, {})
    entry = defaults.get("gripper") if isinstance(defaults, dict) else None
    if _is_registry_eef_entry(entry):
        return copy.deepcopy(entry)
    return copy.deepcopy(_BUILTIN_GRIPPER_DEFAULT)


def has_explicit_registry_entry(eef_type: str) -> bool:
    registry = _load_registry()
    return _is_registry_eef_entry(registry.get(eef_type.strip()))


def resolve_registry_entry(eef_type: str) -> Optional[Dict[str, Any]]:
    """Resolve EEF registry entry; unregistered grippers fall back to _defaults.gripper."""
    normalized = eef_type.strip()
    if not normalized or normalized == "none":
        return None

    registry = _load_registry()
    entry = registry.get(normalized)
    if _is_registry_eef_entry(entry):
        return copy.deepcopy(entry)

    if _is_probable_hand_type(normalized):
        return None

    print(
        f"[INFO] EEF type '{normalized}' not in eef_control_registry; "
        "using default gripper side template"
    )
    return _gripper_default_entry(registry)


def _apply_side_placeholders(content: str, side: str) -> str:
    side_prefix = f"{side}_"
    return (
        content.replace("{side_}", side_prefix)
        .replace("{side}", side)
    )


def _load_side_template_content_from_entry(entry: Dict[str, Any]) -> Optional[str]:
    package = entry["package"]
    template_rel = entry["template"]
    template_path = os.path.join(get_package_share_directory(package), template_rel)
    if not os.path.isfile(template_path):
        print(f"[WARN] EEF control template not found: {template_path}")
        return None
    with open(template_path, "r", encoding="utf-8") as handle:
        return handle.read()


def _controller_key_for_category(side: str, category: str) -> str:
    if category == "hand":
        return f"{side}_hand_controller"
    return f"{side}_gripper_controller"


def _apply_registry_params(fragment: Dict[str, Any], side: str, entry: Dict[str, str]) -> None:
    extra_params = entry.get("params")
    if not extra_params or not isinstance(extra_params, dict):
        return
    controller_key = _controller_key_for_category(side, entry.get("category", "gripper"))
    controller = fragment.get(controller_key)
    if not isinstance(controller, dict):
        return
    params = controller.setdefault("ros__parameters", {})
    if isinstance(params, dict):
        params.update(extra_params)


def _side_fragment(eef_type: str, side: str) -> Dict[str, Any]:
    registry_entry = resolve_registry_entry(eef_type)
    if not registry_entry:
        return {}
    raw = _load_side_template_content_from_entry(registry_entry)
    if not raw:
        return {}
    side_yaml = _apply_side_placeholders(raw, side)
    try:
        fragment = yaml.safe_load(side_yaml) or {}
    except yaml.YAMLError as exc:
        print(f"[WARN] Failed to parse side template for {eef_type} ({side}): {exc}")
        return {}
    _apply_registry_params(fragment, side, registry_entry)
    return fragment


def _enrich_fragment_from_robot_type_config(
    fragment: Dict[str, Any],
    side: str,
    eef_type: str,
    robot_name: str,
    category: str,
) -> Dict[str, Any]:
    """Merge per-type ros2_control params from the robot package (symmetric load)."""
    if not fragment or not robot_name or category != "gripper":
        return fragment

    from .robot_utils import load_robot_config

    cfg, _, _meta = load_robot_config(
        robot_name,
        "ros2_control",
        eef_type,
        control_left=eef_type,
        control_right=eef_type,
        yaml_only=True,
    )
    if not cfg:
        return fragment

    side_extract = _extract_side_from_robot_yaml(cfg, side, category)
    if not side_extract:
        return fragment

    controller_key = _controller_key_for_category(side, category)
    side_controller = side_extract.get(controller_key)
    fragment_controller = fragment.get(controller_key)
    if isinstance(side_controller, dict) and isinstance(fragment_controller, dict):
        side_params = side_controller.get("ros__parameters", {})
        if isinstance(side_params, dict):
            params = fragment_controller.setdefault("ros__parameters", {})
            if isinstance(params, dict):
                params.update(side_params)

    extra = side_extract.get("joint_state_extra_joints")
    if extra:
        fragment["joint_state_extra_joints"] = extra

    return fragment


def _extract_side_from_robot_yaml(config: Dict[str, Any], side: str, category: str) -> Dict[str, Any]:
    """Fallback: extract one side from a symmetric robot package type yaml."""
    fragment: Dict[str, Any] = {}
    if category == "gripper":
        key = f"{side}_gripper_controller"
        if key in config:
            fragment[key] = copy.deepcopy(config[key])
        fragment.setdefault("joint_state_extra_joints", [f"{side}_gripper_joint"])
    elif category == "hand":
        key = f"{side}_hand_controller"
        if key in config:
            fragment[key] = copy.deepcopy(config[key])
        jsb = config.get("joint_state_broadcaster", {}).get("ros__parameters", {})
        joints = jsb.get("joints", [])
        prefix = f"{side}_hand_"
        extra = [j for j in joints if isinstance(j, str) and j.startswith(prefix)]
        if extra:
            fragment["joint_state_extra_joints"] = extra
    return fragment


def _merge_joint_state_lists(
    fragments: List[Dict[str, Any]],
    base_config: Optional[Dict[str, Any]] = None,
) -> List[str]:
    joints = _extract_base_joint_state_joints(base_config or {})
    if not joints:
        print(
            "[WARN] Could not derive base joints from robot config for "
            "joint_state_broadcaster compose; using EEF joints only"
        )
    seen = set(joints)
    for fragment in fragments:
        for joint in fragment.get("joint_state_extra_joints", []) or []:
            if joint not in seen:
                joints.append(joint)
                seen.add(joint)
    return joints


def compose_control_config(
    left_type: str,
    right_type: str,
    robot_name: str = "",
    base_config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build merged control yaml dict from per-side EEF types."""
    composed: Dict[str, Any] = {}
    fragments: List[Dict[str, Any]] = []

    for side, eef_type in (("left", left_type), ("right", right_type)):
        if not eef_type or eef_type == "none" or _is_passive_eef_type(eef_type):
            continue
        fragment = _side_fragment(eef_type, side)
        entry = resolve_registry_entry(eef_type)
        category = entry.get("category", "gripper") if entry else "gripper"
        if fragment and robot_name and not has_explicit_registry_entry(eef_type):
            fragment = _enrich_fragment_from_robot_type_config(
                fragment, side, eef_type, robot_name, category
            )
        if not fragment and robot_name:
            # Fallback to robot package symmetric yaml extraction
            from .robot_utils import load_robot_config

            cfg, _, _meta = load_robot_config(
                robot_name, "ros2_control", eef_type, yaml_only=True
            )
            if cfg:
                entry = resolve_registry_entry(eef_type)
                category = entry.get("category", "gripper") if entry else "gripper"
                fragment = _extract_side_from_robot_yaml(cfg, side, category)
        if fragment:
            composed = _deep_merge_dicts(composed, fragment)
            fragments.append(fragment)

    if fragments:
        cm = composed.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
        for key in composed:
            if key.endswith("_gripper_controller"):
                cm[key] = {"type": "adaptive_gripper_controller/AdaptiveGripperController"}
            elif key.endswith("_hand_controller"):
                cm[key] = {"type": "basic_joint_controller/BasicJointController"}

        joint_list = _merge_joint_state_lists(fragments, base_config)
        composed["joint_state_broadcaster"] = {
            "ros__parameters": {
                "joints": joint_list,
                "interfaces": ["position", "velocity", "effort"],
            }
        }
        # Internal compose metadata; must not appear in ros2_control params file.
        composed.pop("joint_state_extra_joints", None)
    return composed


def is_compose_asymmetric(control_left: str, control_right: str) -> bool:
    """True when per-side compose is required (incl. one bare arm + one EEF)."""
    return control_left.strip() != control_right.strip()


def resolve_compose_type_key(
    launch_type: str,
    control_left: str,
    control_right: str,
) -> Tuple[str, str, str]:
    """
    Returns (effective_type_for_yaml, control_left, control_right).
    When asymmetric, effective_type is empty and compose uses left/right.
    """
    left = control_left.strip()
    right = control_right.strip()
    lt = launch_type.strip()

    if is_compose_asymmetric(left, right):
        return "", left, right

    if lt:
        return lt, left or lt, right or lt

    if left:
        return left, left, left

    return "", "", ""
