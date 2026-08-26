cmake_minimum_required(VERSION 3.8)
project(hik_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-Wall -Werror)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME}_safety STATIC
  src/camera_safety.cpp
)
set_target_properties(${PROJECT_NAME}_safety PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(${PROJECT_NAME}_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(HIK_MVS_LIBRARY_DIR "" CACHE PATH
  "Optional directory containing installed MVS SDK libraries"
)
set(HIK_MVS_SEARCH_PATHS)
if(HIK_MVS_LIBRARY_DIR)
  list(APPEND HIK_MVS_SEARCH_PATHS "${HIK_MVS_LIBRARY_DIR}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  list(APPEND HIK_MVS_SEARCH_PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64"
    "/opt/MVS/lib/64"
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  list(APPEND HIK_MVS_SEARCH_PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64"
    "/opt/MVS/lib/aarch64"
  )
endif()

set(HIK_MVS_LIBRARY_NAMES
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)
set(HIK_MVS_LIBRARIES)
set(HIK_MVS_MISSING_LIBRARIES)

foreach(library_name IN LISTS HIK_MVS_LIBRARY_NAMES)
  string(MAKE_C_IDENTIFIER "${library_name}" library_id)
  set(library_variable "HIK_MVS_${library_id}_LIBRARY")
  find_library(${library_variable}
    NAMES "${library_name}"
    HINTS ${HIK_MVS_SEARCH_PATHS}
  )

  if(${library_variable})
    list(APPEND HIK_MVS_LIBRARIES "${${library_variable}}")
  else()
    list(APPEND HIK_MVS_MISSING_LIBRARIES "${library_name}")
  endif()
endforeach()

if(NOT HIK_MVS_MISSING_LIBRARIES)
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/hik_camera_node.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    hikSDK/include
  )
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_safety
    ${HIK_MVS_LIBRARIES}
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN hik_camera::HikCameraNode
    EXECUTABLE ${PROJECT_NAME}_node
  )
else()
  message(WARNING
    "MVS SDK libraries not found (${HIK_MVS_MISSING_LIBRARIES}). "
    "Skipping the real camera executable; compile checks and SDK-independent "
    "tests remain available."
  )

  add_library(${PROJECT_NAME}_compile_check OBJECT
    src/hik_camera_node.cpp
  )
  target_include_directories(${PROJECT_NAME}_compile_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    hikSDK/include
  )
  ament_target_dependencies(${PROJECT_NAME}_compile_check
    camera_info_manager
    image_transport
    rcl_interfaces
    rclcpp
    rclcpp_components
    sensor_msgs
  )
endif()

install(
  TARGETS ${PROJECT_NAME}_safety
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(camera_safety_test
    test/camera_safety_test.cpp
  )
  if(TARGET camera_safety_test)
    target_compile_definitions(camera_safety_test PRIVATE
      HIK_CAMERA_UNVERIFIED_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/camera_info.yaml"
    )
    target_link_libraries(camera_safety_test
      ${PROJECT_NAME}_safety
    )
    ament_target_dependencies(camera_safety_test
      camera_info_manager
      rclcpp
    )
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
