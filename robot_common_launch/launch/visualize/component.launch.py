from launch.actions import DeclareLaunchArgument
from robot_common_launch import create_visualization_launch_description


def generate_launch_description():
    # 添加额外的 direction 参数
    additional_args = [
        DeclareLaunchArgument(
            'direction',
            default_value='',
            description='Direction of the hand (empty means no direction parameter passed to xacro)'
        )
    ]
    
    return create_visualization_launch_description(
        robot_param_name='robot',
        robot_default_value='agibot_g1',
        xacro_filename='component.xacro',
        rviz_config_name='manipulator.rviz',
        additional_args=additional_args
    )