cmake_minimum_required(VERSION 3.16)
project(orin_hardware_evidence LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(orin_environment_preflight_core
  src/orin_environment_preflight.cpp
)
target_include_directories(orin_environment_preflight_core PUBLIC include)

if(MSVC)
  target_compile_options(orin_environment_preflight_core PRIVATE /W4 /utf-8)
else()
  target_compile_options(orin_environment_preflight_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(orin_environment_preflight
  src/main.cpp
)
target_link_libraries(orin_environment_preflight PRIVATE orin_environment_preflight_core)

include(CTest)
if(BUILD_TESTING)
  set(MVS_TEST_ROOT "${CMAKE_CURRENT_BINARY_DIR}/mvs_test_fixtures")
  set(MVS_TEST_REPO "${MVS_TEST_ROOT}/repo")
  set(MVS_TEST_REPO_LIB "${MVS_TEST_REPO}/src/ros2-hik-camera/hikSDK/lib/arm64")
  set(MVS_TEST_OPT_LIB "${MVS_TEST_ROOT}/opt/MVS/lib/aarch64")
  set(MVS_TEST_EXPLICIT_LIB "${MVS_TEST_ROOT}/explicit")
  foreach(directory IN ITEMS "${MVS_TEST_REPO_LIB}" "${MVS_TEST_OPT_LIB}" "${MVS_TEST_EXPLICIT_LIB}")
    file(MAKE_DIRECTORY "${directory}")
    foreach(library IN ITEMS FormatConversion MediaProcess MvCameraControl MVRender MvUsb3vTL)
      file(WRITE "${directory}/lib${library}.so" "test fixture; not an SDK binary\n")
    endforeach()
  endforeach()

  add_executable(orin_environment_preflight_test
    test/orin_environment_preflight_test.cpp
  )
  target_compile_definitions(orin_environment_preflight_test PRIVATE
    MVS_TEST_REPO="${MVS_TEST_REPO}"
    MVS_TEST_OPT_LIB="${MVS_TEST_OPT_LIB}"
    MVS_TEST_EXPLICIT_LIB="${MVS_TEST_EXPLICIT_LIB}"
  )
  target_link_libraries(orin_environment_preflight_test PRIVATE orin_environment_preflight_core)
  add_test(NAME orin_environment_preflight_test COMMAND orin_environment_preflight_test)
endif()
