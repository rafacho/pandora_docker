cmake_minimum_required(VERSION 3.8...3.28)
project(pandora_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HrefCommand.msg"
  "msg/WheelContacts.msg"
  "srv/IkSrv.srv"
  DEPENDENCIES std_msgs ros_gz_interfaces
)

ament_package()
