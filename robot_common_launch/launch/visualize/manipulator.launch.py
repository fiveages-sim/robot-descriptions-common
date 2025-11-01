from robot_common_launch import create_visualization_launch_description


def generate_launch_description():
    return create_visualization_launch_description(
        robot_param_name='robot',
        robot_default_value='piper',
        xacro_filename='robot.xacro',
        rviz_config_name='manipulator.rviz'
    )