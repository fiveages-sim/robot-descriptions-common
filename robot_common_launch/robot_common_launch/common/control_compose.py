"""Compose ros2_control yaml fragments from per-side end-effector templates."""

from __future__ import annotations

import copy
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

_REGISTRY_CACHE: Optional[Dict[str, Dict[str, str]]] = None

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


def _apply_side_placeholders(content: str, side: str) -> str:
    side_prefix = f"{side}_"
    return (
        content.replace("{side_}", side_prefix)
        .replace("{side}", side)
    )


def _load_side_template_content(eef_type: str) -> Optional[str]:
    registry = _load_registry()
    entry = registry.get(eef_type)
    if not entry:
        return None
    package = entry["package"]
    template_rel = entry["template"]
    template_path = os.path.join(get_package_share_directory(package), template_rel)
    if not os.path.isfile(template_path):
        print(f"[WARN] EEF control template not found: {template_path}")
        return None
    with open(template_path, "r", encoding="utf-8") as handle:
        return handle.read()


def _side_fragment(eef_type: str, side: str) -> Dict[str, Any]:
    raw = _load_side_template_content(eef_type)
    if not raw:
        return {}
    side_yaml = _apply_side_placeholders(raw, side)
    try:
        return yaml.safe_load(side_yaml) or {}
    except yaml.YAMLError as exc:
        print(f"[WARN] Failed to parse side template for {eef_type} ({side}): {exc}")
        return {}


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
    registry = _load_registry()
    composed: Dict[str, Any] = {}
    fragments: List[Dict[str, Any]] = []

    for side, eef_type in (("left", left_type), ("right", right_type)):
        if not eef_type or eef_type == "none":
            continue
        fragment = _side_fragment(eef_type, side)
        if not fragment and robot_name:
            # Fallback to robot package symmetric yaml extraction
            from .robot_utils import load_robot_config, get_robot_package_path

            cfg, _ = load_robot_config(robot_name, "ros2_control", eef_type)
            if cfg:
                category = registry.get(eef_type, {}).get("category", "gripper")
                fragment = _extract_side_from_robot_yaml(cfg, side, category)
        if fragment:
            composed = _deep_merge_dicts(composed, fragment)
            fragments.append(fragment)

    if fragments:
        cm = composed.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
        for side, eef_type in (("left", left_type), ("right", right_type)):
            if not eef_type or eef_type == "none":
                continue
            category = registry.get(eef_type, {}).get("category", "gripper")
            if category == "hand":
                cm[f"{side}_hand_controller"] = {
                    "type": "basic_joint_controller/BasicJointController",
                }
            else:
                cm[f"{side}_gripper_controller"] = {
                    "type": "adaptive_gripper_controller/AdaptiveGripperController",
                }

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
