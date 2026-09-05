cmake_minimum_required(VERSION 3.10)
project(serical_device_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

# ------------------------------------------------------------------
# OpenCV 4.10 is installed under /usr/local (JEP build). Use it directly
# (headers + libs) to avoid version mismatch with the distro OpenCV.
# ------------------------------------------------------------------
set(OPENCV_410_ROOT "/usr/local")
set(OPENCV_410_INCLUDE "${OPENCV_410_ROOT}/include/opencv4")
set(OPENCV_410_LIBS
  "${OPENCV_410_ROOT}/lib/libopencv_core.so"
  "${OPENCV_410_ROOT}/lib/libopencv_imgproc.so"
  "${OPENCV_410_ROOT}/lib/libopencv_calib3d.so"
  "${OPENCV_410_ROOT}/lib/libopencv_dnn.so"
  "${OPENCV_410_ROOT}/lib/libopencv_features2d.so"
  "${OPENCV_410_ROOT}/lib/libopencv_imgcodecs.so"
)

# ------------------------------------------------------------------
# awakening auto_aim core: minimal vendored compile (traditional CV)
# third/awakening is a read-only reference, not part of this repo.
# ------------------------------------------------------------------
get_filename_component(AWAKENING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third/awakening" ABSOLUTE)
set(AWAKENING_GEN "${CMAKE_CURRENT_BINARY_DIR}/awakening_gen")
file(MAKE_DIRECTORY "${AWAKENING_GEN}")
file(READ "${AWAKENING_ROOT}/src/param_deliver.h.in" _PD_TEMPLATE)
string(REPLACE "@CMAKE_CURRENT_SOURCE_DIR@" "${AWAKENING_ROOT}" _PD_CONTENT "${_PD_TEMPLATE}")
file(WRITE "${AWAKENING_GEN}/param_deliver.h" "${_PD_CONTENT}")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

set(AWAKENING_SOURCES
  "${AWAKENING_ROOT}/src/tasks/auto_aim/armor_control/very_aimer.cpp"
  "${AWAKENING_ROOT}/src/tasks/auto_aim/armor_detect/armor_detector.cpp"
  "${AWAKENING_ROOT}/src/tasks/auto_aim/armor_detect/armor_infer.cpp"
  "${AWAKENING_ROOT}/src/tasks/auto_aim/armor_track/armor_target.cpp"
  "${AWAKENING_ROOT}/src/tasks/auto_aim/armor_track/armor_tracker.cpp"
)
add_library(awakening_auto_aim STATIC ${AWAKENING_SOURCES})
set_target_properties(awakening_auto_aim PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(awakening_auto_aim PUBLIC
  "${AWAKENING_ROOT}/src"
  "${AWAKENING_ROOT}/3rdparty"
  "${AWAKENING_GEN}"
  "${OPENCV_410_INCLUDE}"
  "${EIGEN3_INCLUDE_DIR}"
  "/usr/include/eigen3"
)
foreach(src ${AWAKENING_SOURCES})
  set_source_files_properties(${src} PROPERTIES
    COMPILE_OPTIONS "-std=c++23;-include;${AWAKENING_ROOT}/src/utils/net_detector/net_detector_base.hpp")
endforeach()
target_link_libraries(awakening_auto_aim PUBLIC
  ${OPENCV_410_LIBS}
  Eigen3::Eigen
  Ceres::ceres
  yaml-cpp
  spdlog::spdlog
)

# ------------------------------------------------------------------
# ROS package library + nodes
# ------------------------------------------------------------------
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_main.cpp
  src/serial_device.cpp
  src/crc.cpp
  src/auto_aim_node.cpp
  vision_pub_node.cpp
  robot_ctrl_node.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${AWAKENING_ROOT}/src"
  "${AWAKENING_ROOT}/3rdparty"
  "${AWAKENING_GEN}"
  "${OPENCV_410_INCLUDE}"
)
target_link_libraries(${PROJECT_NAME} awakening_auto_aim)
set_source_files_properties(src/auto_aim_node.cpp PROPERTIES COMPILE_OPTIONS "-std=c++23")
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_components sensor_msgs message_filters auto_aim_interfaces)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::VisionPub
  EXECUTABLE vision_pub_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::RobotCtrlSub
  EXECUTABLE robot_ctrl_main
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::AutoAimNode
  EXECUTABLE auto_aim_node
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_auto_package()