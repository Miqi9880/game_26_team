cmake_minimum_required(VERSION 3.8)
project(auto_aim_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/preflight_analyzer.cpp
  src/ros_input_preflight_node.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  auto_aim_interfaces
  rclcpp
  sensor_msgs
)

add_executable(ros_input_preflight src/ros_input_preflight_main.cpp)
target_compile_options(ros_input_preflight PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ros_input_preflight ${PROJECT_NAME})
ament_target_dependencies(ros_input_preflight rclcpp)

install(TARGETS ${PROJECT_NAME} ros_input_preflight
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(preflight_analyzer_test test/preflight_analyzer_test.cpp)
  target_compile_options(preflight_analyzer_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(preflight_analyzer_test ${PROJECT_NAME})

  ament_add_gtest(fake_ros_publishers_test test/fake_ros_publishers_test.cpp)
  target_compile_options(fake_ros_publishers_test PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(fake_ros_publishers_test ${PROJECT_NAME})
  ament_target_dependencies(fake_ros_publishers_test
    auto_aim_interfaces
    rclcpp
    sensor_msgs
  )

  ament_add_gtest(process_contract_test test/process_contract_test.cpp)
  target_compile_options(process_contract_test PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(process_contract_test PRIVATE
    PREFLIGHT_EXECUTABLE_PATH="$<TARGET_FILE:ros_input_preflight>"
  )
  add_dependencies(process_contract_test ros_input_preflight)
  ament_target_dependencies(process_contract_test
    auto_aim_interfaces
    rclcpp
    sensor_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(auto_aim_interfaces rclcpp sensor_msgs)
ament_export_libraries(${PROJECT_NAME})
ament_package()
