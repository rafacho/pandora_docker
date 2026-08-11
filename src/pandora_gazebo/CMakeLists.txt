cmake_minimum_required(VERSION 3.8...3.28)
project(pandora_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
