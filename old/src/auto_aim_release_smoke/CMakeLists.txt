cmake_minimum_required(VERSION 3.8)
project(auto_aim_release_smoke)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}_core
  src/release_smoke.cpp
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/run_release_smoke.sh
  DESTINATION lib/${PROJECT_NAME}
)

# These files used to be source-tree-only operational dependencies.  Install
# an explicit, reviewed set so a release smoke run can execute from the
# install tree after the source directory has been moved out of the way.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/__init__.py
  DESTINATION share/${PROJECT_NAME}/tools
)
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/auto_aim_qualification
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/offline_evidence_report
  DESTINATION share/${PROJECT_NAME}/tools
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../docs/
  DESTINATION share/${PROJECT_NAME}/docs
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../auto_aim_ros2/test/data/model_profile_test.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../auto_aim_ros2/test/data/pnp_test_config.yaml
  DESTINATION share/${PROJECT_NAME}/fixtures
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(release_smoke_test test/release_smoke_test.cpp)
  target_link_libraries(release_smoke_test ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
