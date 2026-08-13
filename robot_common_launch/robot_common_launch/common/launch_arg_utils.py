"""
Launch argument utilities: prefixed xacro_/hardware_/control_ routing and robot profile.
"""

from __future__ import annotations

import ast
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml

XACRO_PREFIX = "xacro_"
HARDWARE_PREFIX = "hardware_"
CONTROL_PREFIX = "control_"

REAL_HARDWARE = frozenset({"real", "real_usb"})

_EEF_XACRO_KEYS = frozenset({"type", "left_type", "right_type"})

_TCP_OFFSET_XACRO_KEYS = (
    "left_tcp_offset_xyz",
    "left_tcp_offset_rpy",
    "right_tcp_offset_xyz",
    "right_tcp_offset_rpy",
)

# launch ``type`` values that select arm topology (left/right/dual),
# not symmetric end-effector keys — must not become left_type/right_type.
_ARM_TOPOLOGY_TYPE_KEYS = frozenset({
    "left",
    "right",
    "dual",
})

_POSE_EXPR_GLOBALS = {
    "pi": math.pi,
    "PI": math.pi,
    "radians": math.radians,
}

def _is_arm_topology_type(type_key: str) -> bool:
    return _strip_eef_key(type_key) in _ARM_TOPOLOGY_TYPE_KEYS


def _strip_eef_key(value: str) -> str:
    return str(value or "").strip()


def _mapping_value_ok(value: Any) -> bool:
    """True when value should be injected into xacro mappings (skip empty list/str)."""
    if value is None:
        return False
    if isinstance(value, (list, tuple, dict)) and len(value) == 0:
        return False
    return bool(str(value).strip()) and str(value).strip() not in ("[]", "{}", "()")


CORE_LAUNCH_KEYS = frozenset({
    "robot",
    "type",
    "left_type",
    "right_type",
    "ft",
    "left_ft",
    "right_ft",
    "tcp_offset_xyz",
    "tcp_offset_rpy",
    "left_tcp_offset_xyz",
    "left_tcp_offset_rpy",
    "right_tcp_offset_xyz",
    "right_tcp_offset_rpy",
    "hardware",
    "use_sim_time",
    "world",
    "world_package",
    "remappings",
    "robot_profile",
    "use_profile_eef",
    "launch_mode",
    "enable_gripper",
    "enable_body",
    "enable_arms_target_manager",
    "ocs2_planning_param_file",
    "ros2_controllers_override",
    "control_variant",
    "chassis",
    "variant",
    "chassis_joints_movable",
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


_PLATFORM_XACRO_KEYS = (
    "chassis",
    "variant",
    "chassis_joints_movable",
)


def create_platform_launch_arguments():
    """Launch args for humanoid / mobile-base xacro (not used by bare manipulators)."""
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "chassis",
            default_value="",
            description="Xacro chassis model key (robot-specific). Empty means no chassis parameter passed to xacro.",
        ),
        DeclareLaunchArgument(
            "chassis_joints_movable",
            default_value="",
            description="Xacro flag for movable chassis joints (true/false). Empty means no chassis_joints_movable parameter passed to xacro.",
        ),
    ]


def _expand_side_pair(sym: str, left: str, right: str) -> Tuple[str, str]:
    """Symmetric shorthand + per-side overrides (same rules as end_effectors)."""
    sym = _strip_eef_key(sym)
    left = _strip_eef_key(left)
    right = _strip_eef_key(right)
    if sym and not left and not right:
        return sym, sym
    if not left:
        left = sym
    if not right:
        right = sym
    return left, right


def _resolve_pose_token(token: str, *, field: str) -> str:
    """Evaluate one xyz/rpy component; allow numbers or pi/PI/radians expressions."""
    raw = str(token).strip()
    if not raw:
        raise ValueError(f"empty component in {field}")

    if raw.startswith("${") and raw.endswith("}"):
        expr = raw[2:-1].strip()
    else:
        expr = raw

    try:
        return str(float(expr))
    except ValueError:
        pass

    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as exc:
        raise ValueError(
            f"unsupported pose expression in {field}: {token!r} "
            f"(allowed: numbers, pi/PI, radians(...), or ${{PI/2}})"
        ) from exc

    allowed_nodes = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Call,
        ast.Name,
        ast.Load,
        ast.Constant,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.Mod,
        ast.UAdd,
        ast.USub,
    )
    for node in ast.walk(tree):
        if not isinstance(node, allowed_nodes):
            raise ValueError(f"disallowed syntax in {field}: {token!r}")
        if isinstance(node, ast.Name) and node.id not in _POSE_EXPR_GLOBALS:
            raise ValueError(f"name {node.id!r} not allowed in {field}")
        if isinstance(node, ast.Call) and not (
            isinstance(node.func, ast.Name) and node.func.id == "radians"
        ):
            raise ValueError(f"only radians() calls are allowed in {field}")

    try:
        value = eval(
            compile(tree, "<tcp_offset>", "eval"),
            {"__builtins__": {}},
            _POSE_EXPR_GLOBALS,
        )
        return str(float(value))
    except ValueError:
        raise
    except Exception as exc:
        raise ValueError(f"failed to evaluate {token!r} in {field}: {exc}") from exc


def resolve_pose_triplet(value: str, *, field: str = "tcp_offset") -> str:
    """Split an xyz/rpy triplet, resolve expressions, return space-separated floats."""
    text = _strip_eef_key(value)
    if not text:
        return ""
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(
            f"{field} must have 3 components, got {len(parts)}: {value!r}"
        )
    return " ".join(_resolve_pose_token(p, field=field) for p in parts)


def normalize_robot_profile(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Normalize robot profile to {xacro, eef, ft, hardware, control} for launch code.

    Schema:
      platform: chassis / variant / chassis_joints_movable — always apply
      defaults.end_effectors: type|left|right — when use_profile_eef
      defaults.ft: type|left|right — always with profile
      defaults.tcp_offset: xyz|rpy|left_*|right_* — always with profile → xacro keys
      hardware: applied when hardware:=real / real_usb
      control.patch: ros2_control overrides
    """
    if not isinstance(data, dict) or not data:
        return {}

    platform = data.get("platform")
    defaults = data.get("defaults")
    has_control = isinstance(data.get("control"), dict)
    has_hardware = isinstance(data.get("hardware"), dict)
    if (
        not isinstance(platform, dict)
        and not isinstance(defaults, dict)
        and not has_control
        and not has_hardware
    ):
        return data

    legacy_hw = data.get("hardware") if isinstance(data.get("hardware"), dict) else {}
    legacy_c = data.get("control") if isinstance(data.get("control"), dict) else {}

    xacro: Dict[str, Any] = {}
    hardware = dict(legacy_hw)
    control: Dict[str, Any] = {}

    plat = platform if isinstance(platform, dict) else {}
    for key in _PLATFORM_XACRO_KEYS:
        if key in plat and _mapping_value_ok(plat[key]):
            xacro[key] = plat[key]

    eef: Dict[str, str] = {}
    ft: Dict[str, str] = {}
    if isinstance(defaults, dict):
        raw_eef = defaults.get("end_effectors")
        if isinstance(raw_eef, dict):
            sym = str(raw_eef.get("type", "") or "").strip()
            left = str(raw_eef.get("left", "") or "").strip()
            right = str(raw_eef.get("right", "") or "").strip()
            if sym:
                eef["type"] = sym
            if left:
                eef["left"] = left
            if right:
                eef["right"] = right

        raw_ft = defaults.get("ft")
        if isinstance(raw_ft, dict):
            sym = str(raw_ft.get("type", "") or "").strip()
            left = str(raw_ft.get("left", "") or "").strip()
            right = str(raw_ft.get("right", "") or "").strip()
            left, right = _expand_side_pair(sym, left, right)
            if left:
                ft["left"] = left
            if right:
                ft["right"] = right

        raw_tcp = defaults.get("tcp_offset")
        if isinstance(raw_tcp, dict):
            sym_xyz = str(raw_tcp.get("xyz", "") or "").strip()
            sym_rpy = str(raw_tcp.get("rpy", "") or "").strip()
            left_xyz, right_xyz = _expand_side_pair(
                sym_xyz,
                str(raw_tcp.get("left_xyz", "") or ""),
                str(raw_tcp.get("right_xyz", "") or ""),
            )
            left_rpy, right_rpy = _expand_side_pair(
                sym_rpy,
                str(raw_tcp.get("left_rpy", "") or ""),
                str(raw_tcp.get("right_rpy", "") or ""),
            )
            if left_xyz:
                xacro["left_tcp_offset_xyz"] = left_xyz
            if left_rpy:
                xacro["left_tcp_offset_rpy"] = left_rpy
            if right_xyz:
                xacro["right_tcp_offset_xyz"] = right_xyz
            if right_rpy:
                xacro["right_tcp_offset_rpy"] = right_rpy

    patch = legacy_c.get("patch")
    if isinstance(patch, dict) and patch:
        control["patch"] = patch

    return {
        "xacro": xacro,
        "eef": eef,
        "ft": ft,
        "hardware": hardware,
        "control": control,
    }


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


def _cli_launch_value(launch_configurations: Dict[str, str], key: str) -> str:
    """Read a launch arg from LaunchConfiguration or ``key:=value`` on the CLI."""
    value = str(launch_configurations.get(key, "") or "").strip()
    if value:
        return value
    prefix = f"{key}:="
    for token in sys.argv:
        if token.startswith(prefix):
            return token[len(prefix) :].strip()
    return ""


def use_profile_end_effectors(launch_configurations: Dict[str, str]) -> bool:
    """When false, ``defaults.end_effectors`` from robot.local.yaml are not applied."""
    val = _cli_launch_value(launch_configurations, "use_profile_eef").lower()
    if val:
        return val not in ("false", "0", "no")
    return True


def _profile_eef_pair(profile: Dict[str, Any]) -> Tuple[str, str]:
    """Left/right type keys from profile ``eef`` section."""
    eef = profile.get("eef")
    if not isinstance(eef, dict) or not eef:
        return "", ""
    return _expand_side_pair(
        str(eef.get("type", "") or ""),
        str(eef.get("left", "") or ""),
        str(eef.get("right", "") or ""),
    )


def resolve_side_eef_types(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    """
    Per-side end-effector type keys for URDF xacro and ros2_control compose.

    Merge order: launch ``type`` / ``left_type`` / ``right_type`` first; profile
    ``defaults.end_effectors`` only when ``use_profile_eef`` is true (default).
    """
    left = _strip_eef_key(_cli_launch_value(launch_configurations, "left_type"))
    right = _strip_eef_key(_cli_launch_value(launch_configurations, "right_type"))
    launch_type = _strip_eef_key(_cli_launch_value(launch_configurations, "type"))

    if launch_type and not left and not right:
        if _is_arm_topology_type(launch_type):
            return "", ""
        return launch_type, launch_type

    if not use_profile_end_effectors(launch_configurations):
        return left, right

    if profile:
        pe_left, pe_right = _profile_eef_pair(profile)
        if not left:
            left = pe_left
        if not right:
            right = pe_right

    return left, right


def resolve_side_ft_types(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Tuple[str, str]:
    """CLI ft/left_ft/right_ft over profile defaults.ft (always, not gated by use_profile_eef)."""
    left = _strip_eef_key(_cli_launch_value(launch_configurations, "left_ft"))
    right = _strip_eef_key(_cli_launch_value(launch_configurations, "right_ft"))
    sym = _strip_eef_key(_cli_launch_value(launch_configurations, "ft"))
    left, right = _expand_side_pair(sym, left, right)

    if profile:
        pft = profile.get("ft") if isinstance(profile.get("ft"), dict) else {}
        if not left:
            left = _strip_eef_key(str(pft.get("left", "") or ""))
        if not right:
            right = _strip_eef_key(str(pft.get("right", "") or ""))
    return left, right


def resolve_tcp_offset(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    """
    Resolve left/right tcp_offset xyz/rpy (CLI > profile), with expression evaluation.

    Returns keys: left_tcp_offset_xyz, left_tcp_offset_rpy, right_tcp_offset_xyz,
    right_tcp_offset_rpy (only non-empty).
    """
    profile_xacro = {}
    if profile:
        px = profile.get("xacro") or {}
        if isinstance(px, dict):
            profile_xacro = px

    sym_xyz = _cli_launch_value(launch_configurations, "tcp_offset_xyz")
    sym_rpy = _cli_launch_value(launch_configurations, "tcp_offset_rpy")
    left_xyz = _cli_launch_value(launch_configurations, "left_tcp_offset_xyz")
    left_rpy = _cli_launch_value(launch_configurations, "left_tcp_offset_rpy")
    right_xyz = _cli_launch_value(launch_configurations, "right_tcp_offset_xyz")
    right_rpy = _cli_launch_value(launch_configurations, "right_tcp_offset_rpy")

    left_xyz, right_xyz = _expand_side_pair(sym_xyz, left_xyz, right_xyz)
    left_rpy, right_rpy = _expand_side_pair(sym_rpy, left_rpy, right_rpy)

    if not left_xyz:
        left_xyz = _strip_eef_key(str(profile_xacro.get("left_tcp_offset_xyz", "") or ""))
    if not right_xyz:
        right_xyz = _strip_eef_key(str(profile_xacro.get("right_tcp_offset_xyz", "") or ""))
    if not left_rpy:
        left_rpy = _strip_eef_key(str(profile_xacro.get("left_tcp_offset_rpy", "") or ""))
    if not right_rpy:
        right_rpy = _strip_eef_key(str(profile_xacro.get("right_tcp_offset_rpy", "") or ""))

    out: Dict[str, str] = {}
    for key, raw in (
        ("left_tcp_offset_xyz", left_xyz),
        ("left_tcp_offset_rpy", left_rpy),
        ("right_tcp_offset_xyz", right_xyz),
        ("right_tcp_offset_rpy", right_rpy),
    ):
        if not raw:
            continue
        out[key] = resolve_pose_triplet(raw, field=key)
    return out


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
    """Alias for :func:`resolve_side_eef_types` (ros2_control compose)."""
    return resolve_side_eef_types(launch_configurations, profile)


def resolve_robot_variant(
    launch_configurations: Dict[str, str],
    profile: Optional[Dict[str, Any]] = None,
) -> str:
    """ros2_control overlay key for ``config/ros2_control/<key>.yaml``.

    Uses ``control_variant`` launch arg (or ``control_variant:=`` prefix form).
    Intentionally separate from xacro ``variant`` (OEM visuals such as sinopec).
    """
    control_variant = _strip_eef_key(_cli_launch_value(launch_configurations, "control_variant"))
    if control_variant:
        return control_variant
    prefixed = extract_prefixed_args(launch_configurations, CONTROL_PREFIX)
    profile_variant = _strip_eef_key(prefixed.get("variant", ""))
    if profile_variant:
        return profile_variant
    return ""


def _apply_ft_and_tcp_to_mappings(
    mappings: Dict[str, str],
    launch_configurations: Dict[str, str],
    profile: Dict[str, Any],
) -> None:
    left_ft, right_ft = resolve_side_ft_types(launch_configurations, profile)
    if left_ft and left_ft.lower() != "none":
        mappings["left_ft"] = left_ft
    if right_ft and right_ft.lower() != "none":
        mappings["right_ft"] = right_ft

    for key, value in resolve_tcp_offset(launch_configurations, profile).items():
        mappings[key] = value


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
        if str(key) in _EEF_XACRO_KEYS or str(key) in _TCP_OFFSET_XACRO_KEYS:
            continue
        if _mapping_value_ok(value):
            mappings[str(key)] = str(value).strip()

    xacro_overrides = extract_prefixed_args(launch_configurations, XACRO_PREFIX)
    for key in ("type", "left_type", "right_type"):
        xacro_overrides.pop(key, None)
    # Prefer first-class ft/tcp_offset resolve over raw xacro_ leftovers for those keys.
    for key in (
        "left_ft",
        "right_ft",
        *_TCP_OFFSET_XACRO_KEYS,
    ):
        xacro_overrides.pop(key, None)
    mappings.update(xacro_overrides)

    if hardware in REAL_HARDWARE:
        profile_hardware = profile.get("hardware") or {}
        if isinstance(profile_hardware, dict):
            for key, value in profile_hardware.items():
                if _mapping_value_ok(value):
                    # Avoid dumping Python list repr; join numeric lists as CSV if needed.
                    if isinstance(value, (list, tuple)):
                        mappings[str(key)] = ",".join(str(v) for v in value)
                    else:
                        mappings[str(key)] = str(value).strip()
        mappings.update(extract_prefixed_args(launch_configurations, HARDWARE_PREFIX))

    _apply_ft_and_tcp_to_mappings(mappings, launch_configurations, profile)

    launch_type = _strip_eef_key(_cli_launch_value(launch_configurations, "type"))
    side_left, side_right = resolve_side_eef_types(launch_configurations, profile)

    if launch_type:
        mappings["type"] = launch_type
    elif use_profile_end_effectors(launch_configurations):
        xt = _strip_eef_key(str((profile.get("eef") or {}).get("type", "") or ""))
        if xt:
            mappings["type"] = xt

    if side_left:
        mappings["left_type"] = side_left
    elif launch_type and not side_left and not side_right:
        mappings.pop("left_type", None)

    if side_right:
        mappings["right_type"] = side_right
    elif launch_type and not side_left and not side_right:
        mappings.pop("right_type", None)

    if hardware == "gz":
        mappings["gazebo"] = "true"

    if "collider" not in mappings:
        mappings["collider"] = "simple"

    launch_variant = _strip_eef_key(_cli_launch_value(launch_configurations, "variant"))
    if launch_variant:
        mappings["variant"] = launch_variant

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
        if key in core or key.startswith((XACRO_PREFIX, HARDWARE_PREFIX)):
            args.append((key, str(value)))
            seen.add(key)

    for prefix in (XACRO_PREFIX, HARDWARE_PREFIX):
        for name, value in extract_prefixed_args(configs, prefix, argv=sys.argv).items():
            full_key = f"{prefix}{name}"
            if full_key not in seen:
                args.append((full_key, value))
                seen.add(full_key)
    return args


def create_eef_side_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "left_type",
            default_value="",
            description="Left end-effector type key (rg75, ag2f90_c, linkerhand_o7, …). "
            "Use with right_type for asymmetric setups; do not pass type:=.",
        ),
        DeclareLaunchArgument(
            "right_type",
            default_value="",
            description="Right end-effector type key. See left_type.",
        ),
    ]


def create_ft_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "ft",
            default_value="",
            description="Symmetric FT sensor key for both arms (none|kwr75_485|kwr75_usb). "
            "Overrides defaults.ft.type from robot_profile when set.",
        ),
        DeclareLaunchArgument(
            "left_ft",
            default_value="",
            description="Left FT sensor key. See ft.",
        ),
        DeclareLaunchArgument(
            "right_ft",
            default_value="",
            description="Right FT sensor key. See ft.",
        ),
    ]


def create_tcp_offset_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "tcp_offset_xyz",
            default_value="",
            description="Symmetric TCP offset xyz (metres) for both arms. "
            "Supports expressions like '${PI/2}' in components (evaluated before xacro).",
        ),
        DeclareLaunchArgument(
            "tcp_offset_rpy",
            default_value="",
            description="Symmetric TCP offset rpy (radians) for both arms. "
            "Supports '${PI/2}', pi/2, radians(90).",
        ),
        DeclareLaunchArgument(
            "left_tcp_offset_xyz",
            default_value="",
            description="Left TCP offset xyz (metres). See tcp_offset_xyz.",
        ),
        DeclareLaunchArgument(
            "left_tcp_offset_rpy",
            default_value="",
            description="Left TCP offset rpy (radians). See tcp_offset_rpy.",
        ),
        DeclareLaunchArgument(
            "right_tcp_offset_xyz",
            default_value="",
            description="Right TCP offset xyz (metres). See tcp_offset_xyz.",
        ),
        DeclareLaunchArgument(
            "right_tcp_offset_rpy",
            default_value="",
            description="Right TCP offset rpy (radians). See tcp_offset_rpy.",
        ),
    ]


def create_robot_profile_launch_arguments():
    from launch.actions import DeclareLaunchArgument

    return [
        DeclareLaunchArgument(
            "robot_profile",
            default_value="",
            description="Path to robot.local.yaml (machine hardware profile)",
        ),
        DeclareLaunchArgument(
            "use_profile_eef",
            default_value="true",
            description="Apply defaults.end_effectors from robot_profile (false for quick_start templates)",
        ),
        *create_eef_side_launch_arguments(),
        *create_ft_launch_arguments(),
        *create_tcp_offset_launch_arguments(),
    ]
