cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vision.msg"
  "msg/RobotCtrl.msg"
  DEPENDENCIES std_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
