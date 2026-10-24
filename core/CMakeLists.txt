cmake_minimum_required(VERSION 3.16)
project(treemapper_core VERSION 0.1.0 LANGUAGES CXX)

# ROS-free core library. C++17, standard library only (Eigen/CGAL may be added by
# later milestones; both exist on the Orin's Ubuntu 20.04 / Noetic stack).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_library(treemapper_core
  src/types.cpp
  src/tree_map.cpp
  src/geometry.cpp
  src/simple_mapper.cpp
)
target_include_directories(treemapper_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(treemapper_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(treemapper_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Offline tooling: frame-file reader + replay runner (not needed by the ROS nodes).
add_library(treemapper_io src/frame_io.cpp)
target_link_libraries(treemapper_io PUBLIC treemapper_core)
target_compile_options(treemapper_io PRIVATE -Wall -Wextra -Wpedantic)
add_executable(treemapper_replay apps/replay_main.cpp)
target_link_libraries(treemapper_replay PRIVATE treemapper_io)
target_compile_options(treemapper_replay PRIVATE -Wall -Wextra -Wpedantic)

option(TREEMAPPER_BUILD_TESTS "Build the core unit tests" ON)
if(TREEMAPPER_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND AND EXISTS /usr/src/googletest/CMakeLists.txt)
    # Ubuntu 20.04 ships googletest as sources only.
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    add_subdirectory(/usr/src/googletest ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
    add_library(GTest::gtest_main ALIAS gtest_main)
  endif()
  add_executable(test_core tests/test_tree_map.cpp tests/test_simple_mapper.cpp)
  target_link_libraries(test_core PRIVATE treemapper_core GTest::gtest_main)
  add_test(NAME test_core COMMAND test_core)
endif()

install(TARGETS treemapper_core treemapper_io ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
