cmake_minimum_required(VERSION 3.8)
project(auto_aim_ros_e2e)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_report src/report.cpp)
target_compile_options(${PROJECT_NAME}_report PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_report PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_report OpenSSL::Crypto)

add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_report)
ament_target_dependencies(${PROJECT_NAME}
  auto_aim_interfaces
  rclcpp
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_report
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/run_ros_message_e2e.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../auto_aim_ros2/test/data/model_profile_test.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../auto_aim_ros2/test/data/pnp_test_config.yaml
  DESTINATION share/${PROJECT_NAME}/fixtures
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(report_test test/report_test.cpp)
  target_link_libraries(report_test ${PROJECT_NAME}_report)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_report)
ament_package()
