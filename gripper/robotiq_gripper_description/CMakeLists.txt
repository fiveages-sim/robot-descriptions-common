cmake_minimum_required(VERSION 3.14)
project(robotiq_gripper_description)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY xacro
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
