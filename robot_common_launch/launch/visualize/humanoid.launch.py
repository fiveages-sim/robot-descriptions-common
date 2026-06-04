from robot_common_launch import (
    create_platform_launch_arguments,
    create_visualization_launch_description,
)


def generate_launch_description():
    return create_visualization_launch_description(
        robot_param_name='robot',
        robot_default_value='unitree_g1',
        xacro_filename='robot.xacro',
        rviz_config_name='humanoid.rviz',
        additional_args=create_platform_launch_arguments(),
    )