cmake_minimum_required(VERSION 3.14)
project(sensor_models)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()