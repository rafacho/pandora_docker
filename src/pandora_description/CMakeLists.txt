cmake_minimum_required(VERSION 3.8...3.28)
project(pandora_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

# materials.xacro is not currently xacro:included by anything (dead file,
# noted in the ROS2-port code review 2026-08-04), kept as-is and installed for
# parity with the ROS1 package contents.
install(
  FILES materials.xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
