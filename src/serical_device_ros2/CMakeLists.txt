cmake_minimum_required(VERSION 3.10)
project(serical_device_ros2)

# 设置 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# clangd 支持
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 安装路径兼容
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 编译警告（可选）
# add_definitions(-Wall -Werror)

#########################################
# ✅ 自动依赖 + 头文件包含
#########################################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

#########################################
# ✅ 构建组件库
#########################################
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_main.cpp
  src/serial_device.cpp
  src/crc.cpp
  vision_pub_node.cpp
  robot_ctrl_node.cpp
)

#########################################
# ✅ 注册组件并自动生成执行文件
#########################################
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::VisionPub
  EXECUTABLE vision_pub_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::RobotCtrlSub
  EXECUTABLE robot_ctrl_main
)

#########################################
# ✅ 安装头文件（如果有 include 文件夹）
#########################################
install(DIRECTORY include/
  DESTINATION include/
)

#########################################
# ✅ 打包
#########################################
ament_auto_package()
