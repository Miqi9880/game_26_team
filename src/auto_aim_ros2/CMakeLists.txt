cmake_minimum_required(VERSION 3.8)
project(auto_aim_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED COMPONENTS calib3d core dnn imgcodecs imgproc videoio)
find_package(yaml-cpp REQUIRED)

# The WSL image used by the team provides the C++ runtime here.  Keep the
# lookup optional so the ROS communication/dry-run package still builds on a
# machine without OpenVINO; the detector then reports a clear runtime error.
if(NOT OpenVINO_DIR AND EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake")
  set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake" CACHE PATH
    "OpenVINO CMake package directory")
endif()
find_package(OpenVINO CONFIG QUIET COMPONENTS Runtime)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/auto_aim_core.cpp
  src/auto_aim_node.cpp
  src/camera_calibration.cpp
  src/offline_pipeline.cpp
  src/pnp_stage.cpp
  src/raw_armor_detector.cpp
  src/ros_image_adapter.cpp
  src/ros_backend.cpp
  src/vision_time_alignment.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)

if(OpenVINO_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE AUTO_AIM_HAS_OPENVINO=1)
  target_link_libraries(${PROJECT_NAME} openvino::runtime)
  message(STATUS "auto_aim_ros2: OpenVINO detector enabled (${OpenVINO_DIR})")
else()
  message(WARNING "auto_aim_ros2: OpenVINO not found; detector smoke will report unavailable")
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::AutoAimNode
  EXECUTABLE auto_aim_node
)

add_executable(auto_aim_dry_run src/auto_aim_dry_run.cpp)
target_link_libraries(auto_aim_dry_run ${PROJECT_NAME})
target_include_directories(auto_aim_dry_run PRIVATE include)
install(TARGETS auto_aim_dry_run
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(auto_aim_detector_smoke src/auto_aim_detector_smoke.cpp)
target_link_libraries(auto_aim_detector_smoke ${PROJECT_NAME})
target_include_directories(auto_aim_detector_smoke PRIVATE include)
install(TARGETS auto_aim_detector_smoke
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(auto_aim_pnp_smoke src/auto_aim_pnp_smoke.cpp)
target_link_libraries(auto_aim_pnp_smoke ${PROJECT_NAME})
target_include_directories(auto_aim_pnp_smoke PRIVATE include)
install(TARGETS auto_aim_pnp_smoke
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(auto_aim_offline src/auto_aim_offline.cpp)
target_link_libraries(auto_aim_offline ${PROJECT_NAME})
target_include_directories(auto_aim_offline PRIVATE include)
install(TARGETS auto_aim_offline
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(auto_aim_camera_calibrate src/auto_aim_camera_calibrate.cpp)
target_link_libraries(auto_aim_camera_calibrate ${PROJECT_NAME})
target_include_directories(auto_aim_camera_calibrate PRIVATE include)
install(TARGETS auto_aim_camera_calibrate
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_auto_add_gtest(
    auto_aim_core_test
    test/auto_aim_core_test.cpp
  )
  ament_auto_add_gtest(
    raw_armor_detector_test
    test/raw_armor_detector_test.cpp
  )
  target_compile_definitions(
    raw_armor_detector_test PRIVATE
    MODEL_PROFILE_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/model_profile_test.yaml"
  )
  ament_auto_add_gtest(
    ros_image_adapter_test
    test/ros_image_adapter_test.cpp
  )
  ament_auto_add_gtest(
    pnp_stage_test
    test/pnp_stage_test.cpp
  )
  ament_auto_add_gtest(
    offline_pipeline_test
    test/offline_pipeline_test.cpp
  )
  ament_auto_add_gtest(
    offline_tracker_replay_test
    test/offline_tracker_replay_test.cpp
  )
  ament_auto_add_gtest(
    offline_predictor_test
    test/offline_predictor_test.cpp
  )
  ament_auto_add_gtest(
    ros_backend_test
    test/ros_backend_test.cpp
  )
  ament_auto_add_gtest(
    vision_time_alignment_test
    test/vision_time_alignment_test.cpp
  )
  ament_auto_add_gtest(
    camera_calibration_test
    test/camera_calibration_test.cpp
  )
  target_compile_definitions(
    camera_calibration_test PRIVATE
    CAMERA_CALIBRATION_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/camera_calibration_test.yaml"
    CAMERA_CALIBRATION_EXECUTABLE_PATH="$<TARGET_FILE:auto_aim_camera_calibrate>"
  )
  target_compile_definitions(
    pnp_stage_test PRIVATE
    PNP_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/pnp_test_config.yaml"
  )
  add_test(
    NAME ros_safety_integration_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/ros_safety_integration_test.py
  )
  # Two isolated rounds start several ROS process groups and deliberately
  # exercise watchdog timeouts.  Give their Python signal/atexit cleanup a
  # realistic budget rather than terminating it before child groups are reaped.
  set_tests_properties(ros_safety_integration_test PROPERTIES TIMEOUT 120)
endif()

ament_auto_package()
