"""Resolve navigation maps and Nav2 params across robot / component / common packages."""

from __future__ import annotations

import glob
import os
from typing import Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory

ROBOT_NAVIGATION_CONFIG_FILENAME = 'navigation.yaml'

# ament package name -> workspace-relative source directory (for pre-build fallback)
_SOURCE_PACKAGE_PATHS: Dict[str, str] = {
    'fiveages_w2_description': os.path.join('src', 'fa-w2-description'),
    'fiveages_w2_common_description': os.path.join('src', 'fa-w2-components'),
    'robot_common_launch': os.path.join(
        'src', 'robot-descriptions-common', 'robot_common_launch'),
}

DEFAULT_MAP_FOLDER = 'warehouse_with_forklifts'
NAV2_DEFAULT_FILENAME = 'nav2_params_isaac_gt.yaml'
NAV2_MAP_ONLY_FILENAME = 'nav2_params_isaac_gt_map_only.yaml'

_nav_config_cache: Dict[str, dict] = {}


def find_map_yaml_in_directory(directory: str) -> Optional[str]:
    """Prefer map.yaml; otherwise first *.yaml in directory (lexicographic)."""
    if not os.path.isdir(directory):
        return None
    primary = os.path.join(directory, 'map.yaml')
    if os.path.isfile(primary):
        return os.path.abspath(primary)
    yamls = sorted(glob.glob(os.path.join(directory, '*.yaml')))
    if yamls:
        return os.path.abspath(yamls[0])
    return None


def get_package_share(pkg_name: str, ws_dir: Optional[str] = None) -> Optional[str]:
    """Return package share directory; fallback to workspace source tree when not installed."""
    if not pkg_name:
        return None
    try:
        return get_package_share_directory(pkg_name)
    except (PackageNotFoundError, Exception):
        pass
    if not ws_dir:
        return None
    rel = _SOURCE_PACKAGE_PATHS.get(pkg_name)
    if not rel:
        return None
    candidate = os.path.join(ws_dir, rel)
    if os.path.isdir(candidate):
        return os.path.abspath(candidate)
    return None


def _dedupe_packages(names: List[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for name in names:
        name = (name or '').strip()
        if not name or name in seen:
            continue
        seen.add(name)
        out.append(name)
    return out


def load_robot_navigation_config(
    robot_name: str = '',
    ws_dir: Optional[str] = None,
) -> dict:
    """
    Load {robot}_description/config/navigation.yaml if present.

    Expected keys: map_config_package, nav_config_package (ament package names).
    """
    robot_name = (robot_name or '').strip()
    if not robot_name:
        return {}

    cache_key = f'{robot_name}:{ws_dir or ""}'
    if cache_key in _nav_config_cache:
        return _nav_config_cache[cache_key]

    robot_pkg = f'{robot_name}_description'
    share = get_package_share(robot_pkg, ws_dir)
    if not share:
        _nav_config_cache[cache_key] = {}
        return {}

    config_path = os.path.join(share, 'config', ROBOT_NAVIGATION_CONFIG_FILENAME)
    if not os.path.isfile(config_path):
        _nav_config_cache[cache_key] = {}
        return {}

    try:
        with open(config_path, encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}
    except OSError:
        raw = {}

    if not isinstance(raw, dict):
        raw = {}

    result = {
        'map_config_package': str(raw.get('map_config_package') or '').strip(),
        'nav_config_package': str(raw.get('nav_config_package') or '').strip(),
    }
    _nav_config_cache[cache_key] = result
    return result


def build_config_search_chain(
    robot_name: str = '',
    config_package: str = '',
    ws_dir: Optional[str] = None,
    *,
    chain_kind: str = 'map',
) -> List[Tuple[str, str]]:
    """
    Build ordered (package_name, share_path) pairs for map or nav2 lookup.

    Order:
      explicit launch arg -> robot config/navigation.yaml -> {robot}_description
      -> robot_common_launch
    """
    robot_name = (robot_name or '').strip()
    robot_nav = load_robot_navigation_config(robot_name, ws_dir)
    yaml_pkg = ''
    if chain_kind == 'nav':
        yaml_pkg = robot_nav.get('nav_config_package', '')
    else:
        yaml_pkg = robot_nav.get('map_config_package', '')

    candidates: List[str] = []
    if config_package:
        candidates.append(config_package.strip())
    if yaml_pkg:
        candidates.append(yaml_pkg)
    if robot_name:
        candidates.append(f'{robot_name}_description')
    candidates.append('robot_common_launch')

    resolved: List[Tuple[str, str]] = []
    seen_paths = set()
    for pkg in _dedupe_packages(candidates):
        share = get_package_share(pkg, ws_dir)
        if not share or share in seen_paths:
            continue
        seen_paths.add(share)
        resolved.append((pkg, share))
    return resolved


def _maps_roots(
    robot_name: str = '',
    map_config_package: str = '',
    ws_dir: Optional[str] = None,
) -> List[Tuple[str, str]]:
    return [
        (pkg, os.path.join(share, 'config', 'maps'))
        for pkg, share in build_config_search_chain(
            robot_name, map_config_package, ws_dir, chain_kind='map')
    ]


def _nav_roots(
    robot_name: str = '',
    nav_config_package: str = '',
    ws_dir: Optional[str] = None,
) -> List[Tuple[str, str]]:
    return [
        (pkg, os.path.join(share, 'config', 'nav2'))
        for pkg, share in build_config_search_chain(
            robot_name, nav_config_package, ws_dir, chain_kind='nav')
    ]


def default_map_yaml(
    ws_dir: Optional[str] = None,
    map_folder: str = DEFAULT_MAP_FOLDER,
) -> str:
    """Default map yaml under robot_common_launch (or source fallback)."""
    common_share = get_package_share('robot_common_launch', ws_dir)
    if common_share:
        default_dir = os.path.join(common_share, 'config', 'maps', map_folder)
        found = find_map_yaml_in_directory(default_dir)
        if found:
            return found
        return os.path.join(default_dir, 'map.yaml')
    return os.path.join('config', 'maps', map_folder, 'map.yaml')


def default_nav2_params_yaml(ws_dir: Optional[str] = None) -> str:
    common_share = get_package_share('robot_common_launch', ws_dir)
    if common_share:
        return os.path.join(common_share, 'config', 'nav2', NAV2_DEFAULT_FILENAME)
    return os.path.join('config', 'nav2', NAV2_DEFAULT_FILENAME)


def resolve_map_yaml(
    map_arg: str,
    *,
    robot_name: str = '',
    map_config_package: str = '',
    ws_dir: Optional[str] = None,
    default_map_yaml_path: Optional[str] = None,
) -> str:
    """
    Resolve map for map_server: absolute yaml path, folder name, or relative path under
    config/maps search chain; else default_map_yaml_path.
    """
    fallback = default_map_yaml_path or default_map_yaml(ws_dir)
    raw = (map_arg or '').strip()
    if not raw:
        return fallback

    expanded = os.path.expanduser(raw)
    if os.path.isfile(expanded):
        return os.path.abspath(expanded)
    if os.path.isdir(expanded):
        found = find_map_yaml_in_directory(expanded)
        if found:
            return found

    for _pkg, root in _maps_roots(robot_name, map_config_package, ws_dir):
        if not os.path.isdir(root):
            continue
        candidate = os.path.normpath(os.path.join(root, expanded))
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
        if os.path.isdir(candidate):
            found = find_map_yaml_in_directory(candidate)
            if found:
                return found
        if not expanded.endswith(('.yaml', '.yml')):
            yaml_only = candidate + '.yaml'
            if os.path.isfile(yaml_only):
                return os.path.abspath(yaml_only)

    chain_hint = ', '.join(pkg for pkg, _ in _maps_roots(robot_name, map_config_package, ws_dir))
    print(
        f'[navigation_config] Could not resolve map {raw!r} under [{chain_hint}]; '
        f'using default:\n  {fallback}'
    )
    return fallback


def resolve_nav2_params(
    *,
    robot_name: str = '',
    nav_config_package: str = '',
    nav2_profile: str = 'default',
    params_file: str = '',
    ws_dir: Optional[str] = None,
    default_params_yaml: Optional[str] = None,
) -> str:
    """Resolve Nav2 params yaml; explicit params_file wins."""
    explicit = (params_file or '').strip()
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))

    fallback = default_params_yaml or default_nav2_params_yaml(ws_dir)
    profile = (nav2_profile or 'default').strip().lower()

    if profile == 'map_only':
        filenames = [NAV2_MAP_ONLY_FILENAME, NAV2_DEFAULT_FILENAME]
    else:
        filenames = [NAV2_DEFAULT_FILENAME]

    for _pkg, nav_dir in _nav_roots(robot_name, nav_config_package, ws_dir):
        if not os.path.isdir(nav_dir):
            continue
        for filename in filenames:
            candidate = os.path.join(nav_dir, filename)
            if os.path.isfile(candidate):
                if profile == 'map_only' and filename == NAV2_DEFAULT_FILENAME:
                    print(
                        '[navigation_config] nav2_profile=map_only but '
                        f'{os.path.join(nav_dir, NAV2_MAP_ONLY_FILENAME)!r} not found; '
                        f'using {candidate!r}.'
                    )
                return os.path.abspath(candidate)

    return fallback


def list_navigation_maps(
    *,
    robot_name: str = '',
    map_config_package: str = '',
    ws_dir: Optional[str] = None,
) -> List[dict]:
    """
    List valid map folder names from the map search chain.

    Returns dicts: id, source (package name), yaml_path, default (bool).
    """
    entries: Dict[str, dict] = {}
    order: List[str] = []

    for pkg, root in _maps_roots(robot_name, map_config_package, ws_dir):
        if not os.path.isdir(root):
            continue
        try:
            names = sorted(os.listdir(root))
        except OSError:
            continue
        for name in names:
            subdir = os.path.join(root, name)
            if not os.path.isdir(subdir):
                continue
            yaml_path = find_map_yaml_in_directory(subdir)
            if not yaml_path:
                continue
            if name not in entries:
                entries[name] = {
                    'id': name,
                    'source': pkg,
                    'yaml_path': yaml_path,
                    'default': False,
                }
                order.append(name)

    if DEFAULT_MAP_FOLDER in entries:
        entries[DEFAULT_MAP_FOLDER]['default'] = True
    elif order:
        entries[order[0]]['default'] = True

    return [entries[name] for name in order]
