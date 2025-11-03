from robot_common_launch import create_visualization_launch_description


def generate_launch_description():
    return create_visualization_launch_description(
        robot_param_name='robot',
        robot_default_value='agibot_g1',
        xacro_filename='component.xacro',
        rviz_config_name='manipulator.rviz'
    )