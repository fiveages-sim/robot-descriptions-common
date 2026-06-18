import launch
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node

# Import robot_common_launch utilities
from robot_common_launch import (
    get_robot_package_path,
    resolve_planning_urdf_file_or_fail,
    create_robot_profile_launch_arguments,
)

# Planning-only launch (no ros2_control); xacro mappings use a fixed hardware placeholder.
_PLANNING_XACRO_HARDWARE = "mock_components"


def _detect_pinocchio_version():
    """Best-effort detection of the installed pinocchio version (string or None).

    Tries ``pkg-config --modversion pinocchio`` first, then scans common
    ``pinocchio/config.hpp`` locations and extracts ``PINOCCHIO_VERSION``.
    """
    import subprocess
    try:
        out = subprocess.check_output(
            ['pkg-config', '--modversion', 'pinocchio'],
            stderr=subprocess.DEVNULL, text=True, timeout=2,
        ).strip()
        if out:
            return out
    except (OSError, subprocess.SubprocessError):
        pass
    import re
    pattern = re.compile(r'#\s*define\s+PINOCCHIO_VERSION\s+"([^"]+)"')
    for hdr in ('/opt/ros/jazzy/include/pinocchio/config.hpp',
                '/opt/ros/kilted/include/pinocchio/config.hpp',
                '/usr/include/pinocchio/config.hpp'):
        try:
            with open(hdr, 'r') as fh:
                for line in fh:
                    m = pattern.search(line)
                    if m:
                        return m.group(1)
        except OSError:
            continue
    return None


def _ocs2_codegen_dir(pkg, sub=''):
    """Resolve CppAD code-gen cache directory for *pkg*.

    Default: ``/tmp/ocs2_<USER>_<UID>/<pkg>/auto_generated[/pinocchio-<ver>]/<sub>``
    Override via env var ``OCS2_CODEGEN_DIR``.

    Living under /tmp means a fresh boot guarantees the cache is rebuilt, so
    system upgrades (e.g. pinocchio) cannot silently leave stale ABI .so.
    Additionally, when pinocchio is detectable on this host, the cache path is
    suffixed with the pinocchio version — apt-upgrading pinocchio thus moves
    the cache to a fresh subdirectory automatically, without a reboot.
    """
    base = os.environ.get('OCS2_CODEGEN_DIR') or \
        f"/tmp/ocs2_{os.environ.get('USER', 'anon')}_{os.getuid()}"
    parts = [base, pkg, 'auto_generated']
    version = _detect_pinocchio_version()
    if version:
        parts.append(f'pinocchio-{version}')
    if sub:
        parts.extend(p for p in sub.split('/') if p)
    return os.path.join(*parts)


def is_wsl():
    """检测是否在 WSL 环境中"""
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def generate_launch_description():
    """
    通用的轮式人形机器人 OCS2 launch文件
    
    使用方法:
    ros2 launch robot_common_launch humanoid_ocs2.launch.py robot_name:=fa_w2
    ros2 launch robot_common_launch humanoid_ocs2.launch.py robot_name:=fa_w2 type:=v1
    ros2 launch robot_common_launch humanoid_ocs2.launch.py robot_name:=fa_w2 task_file:=task_custom
    ros2 launch robot_common_launch humanoid_ocs2.launch.py robot_name:=fa_w2 mode:=DUAL_COUPLED
    """
    
    # 机器人名称参数
    robot_name = launch.actions.DeclareLaunchArgument(
        name='robot_name',
        default_value='fiveages_w2',
        description='Name of the humanoid robot (e.g., fa_w2)'
    )

    # 机器人类型参数
    robot_type = launch.actions.DeclareLaunchArgument(
        name='type',
        default_value='',
        description='Robot type/variant. If empty, uses default configuration.'
    )

    debug = launch.actions.DeclareLaunchArgument(
        name='debug',
        default_value='false',
        description='Whether to enable debug mode'
    )

    # 手柄控制参数
    enable_joystick = launch.actions.DeclareLaunchArgument(
        name='enable_joystick',
        default_value='false',
        description='Whether to enable joystick control'
    )

    joystick_device = launch.actions.DeclareLaunchArgument(
        name='joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path (e.g., /dev/input/js0)'
    )

    # OCS2任务文件参数
    task_file = launch.actions.DeclareLaunchArgument(
        name='task_file',
        default_value='humanoid',
        description='OCS2 task file name (without .info extension)'
    )

    # 初始模式参数
    initial_mode = launch.actions.DeclareLaunchArgument(
        name='mode',
        default_value='DUAL_INDEPENDENT',
        description='Initial humanoid mode: DUAL_INDEPENDENT, DUAL_COUPLED, LEFT_ONLY, RIGHT_ONLY, BODY_TRACKING'
    )

    # RViz参数
    rviz = launch.actions.DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    def launch_setup(context, *args, **kwargs):
        robot_name_value = context.launch_configurations['robot_name']
        type_value = context.launch_configurations['type']
        debug_value = context.launch_configurations['debug']
        enable_joystick_value = context.launch_configurations['enable_joystick']
        joystick_device_value = context.launch_configurations['joystick_device']
        task_file_value = context.launch_configurations['task_file']
        initial_mode_value = context.launch_configurations['mode']
        rviz_value = context.launch_configurations['rviz']
        
        # 确定终端前缀
        if is_wsl():
            terminal_prefix = "xterm -e"
            print("🖥️ Detected WSL, using xterm as terminal")
        else:
            terminal_prefix = "gnome-terminal --"
            print("🖥️ Using gnome-terminal for separate windows")
        
        # 生成带类型的机器人标识符
        robot_identifier = robot_name_value
        if type_value and type_value.strip():
            robot_identifier = f"{robot_name_value}_{type_value}"
            print(f"🤖 Launching Wheel Humanoid OCS2 for robot: {robot_name_value} (type: {type_value})")
        else:
            print(f"🤖 Launching Wheel Humanoid OCS2 for robot: {robot_name_value} (default type)")
        
        print(f"🎯 Initial mode: {initial_mode_value}")
        
        # 手柄配置信息
        if enable_joystick_value == 'true':
            print(f"🎮 Joystick control enabled: {joystick_device_value}")
        else:
            print(f"🎮 Joystick control disabled")
        
        # 获取机器人包路径
        robot_pkg_path = get_robot_package_path(robot_name_value)
        if robot_pkg_path is None:
            print(f"❌ Error: Could not find {robot_name_value}_description package")
            return []
        
        # 获取 xacro 生成的 planning URDF
        urdf_file_value = resolve_planning_urdf_file_or_fail(
            robot_name_value,
            context.launch_configurations,
            _PLANNING_XACRO_HARDWARE,
            planning_scope="full",
        )
        if not urdf_file_value:
            print(f"❌ Error: No xacro planning URDF for robot '{robot_name_value}'")
            return []
        print(f"📁 URDF (xacro): {urdf_file_value}")
        
        # 读取URDF内容
        with open(urdf_file_value, 'r') as f:
            robot_description = f.read()
        
        # 构建任务文件路径
        task_file_path = os.path.join(
            robot_pkg_path,
            'config', 'ocs2', f'{task_file_value}.info'
        )
        print(f"📁 Task file: {task_file_path}")
        
        try:
            # lib folder路径
            lib_folder_value = _ocs2_codegen_dir('ocs2_wheel_humanoid', robot_identifier)
            print(f"📁 Lib folder: {lib_folder_value}")
        except Exception as e:
            print(f"❌ Error: Could not find ocs2_wheel_humanoid package: {e}")
            return []

        nodes_to_return = []

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
        nodes_to_return.append(robot_state_publisher)

        # MPC Node - 在独立终端中运行（debug 模式时）
        if debug_value == 'true':
            mpc_node = Node(
                package='ocs2_wheel_humanoid_ros',
                executable='wheel_humanoid_mpc',
                name='wheel_humanoid_mpc',
                prefix=terminal_prefix,
                parameters=[{
                    'urdf_file': urdf_file_value,
                    'task_file': task_file_path,
                    'library_folder': lib_folder_value,
                }],
                output='screen'
            )
        else:
            mpc_node = Node(
                package='ocs2_wheel_humanoid_ros',
                executable='wheel_humanoid_mpc',
                name='wheel_humanoid_mpc',
                parameters=[{
                    'urdf_file': urdf_file_value,
                    'task_file': task_file_path,
                    'library_folder': lib_folder_value,
                }],
                output='screen'
            )
        nodes_to_return.append(mpc_node)

        # Dummy Simulation Node - 在独立终端中运行
        dummy_node = Node(
            package='ocs2_wheel_humanoid_ros',
            executable='wheel_humanoid_dummy',
            name='wheel_humanoid_dummy',
            prefix=terminal_prefix,
            parameters=[{
                'urdf_file': urdf_file_value,
                'task_file': task_file_path,
                'library_folder': lib_folder_value,
            }],
            output='screen'
        )
        nodes_to_return.append(dummy_node)

        # Target Publisher Node
        target_node = Node(
            package='ocs2_wheel_humanoid_ros',
            executable='wheel_humanoid_target',
            name='wheel_humanoid_target',
            parameters=[{
                'urdf_file': urdf_file_value,
                'task_file': task_file_path,
                'library_folder': lib_folder_value,
                'initial_mode': initial_mode_value,
            }],
            output='screen'
        )
        nodes_to_return.append(target_node)

        # RViz
        if rviz_value == 'true':
            try:
                rviz_config = os.path.join(
                    get_package_share_directory('ocs2_wheel_humanoid_ros'),
                    'config', 'wheel_humanoid.rviz'
                )
                rviz_node = Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
                nodes_to_return.append(rviz_node)
                print("✅ RViz configured")
            except Exception as e:
                print(f"⚠️ Warning: Could not configure RViz: {e}")

        # Joy Node (if joystick enabled)
        if enable_joystick_value == 'true':
            joy_node = Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'dev': joystick_device_value,
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0
                }],
                output='screen'
            )
            nodes_to_return.append(joy_node)
            print("✅ Joy node added for joystick control")

        print(f"✅ Wheel Humanoid OCS2 configured with {len(nodes_to_return)} nodes")
        return nodes_to_return

    return LaunchDescription([
        robot_name,
        robot_type,
        debug,
        enable_joystick,
        joystick_device,
        task_file,
        initial_mode,
        rviz,
        *create_robot_profile_launch_arguments(),
        OpaqueFunction(function=launch_setup)
    ])
