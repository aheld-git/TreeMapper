cmake_minimum_required(VERSION 3.16)
project(corridor_nav VERSION 0.1.0 LANGUAGES CXX)

# ROS-free navigator library. C++17, standard library + treemapper_core only.
# Sources and tests are globbed: adding src/*.cpp or tests/*.cpp needs no edit here.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# The core is used as a library only; its own tests are not built here.
set(TREEMAPPER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core ${CMAKE_BINARY_DIR}/treemapper_core EXCLUDE_FROM_ALL)

file(GLOB CN_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(corridor_nav_core ${CN_SOURCES})
target_include_directories(corridor_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(corridor_nav_core PUBLIC treemapper_core)
target_compile_options(corridor_nav_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(corridor_nav_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

enable_testing()
find_package(GTest QUIET)
if(NOT GTest_FOUND AND EXISTS /usr/src/googletest/CMakeLists.txt)
  # Ubuntu 20.04 ships googletest as sources only.
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  add_subdirectory(/usr/src/googletest ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()
file(GLOB CN_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
add_executable(test_nav ${CN_TESTS})
target_link_libraries(test_nav PRIVATE corridor_nav_core GTest::gtest_main)
target_compile_options(test_nav PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME test_nav COMMAND test_nav)
