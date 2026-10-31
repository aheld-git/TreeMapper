cmake_minimum_required(VERSION 3.0.2)
project(corridor_nav_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros
  treemapper_msgs corridor_nav_msgs)
find_package(yaml-cpp REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp roslib std_msgs std_srvs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros
               treemapper_msgs corridor_nav_msgs message_runtime)

# --- the ROS-free navigator (nav/corridor_nav) and, through it, the TreeMapper core, built in-tree ----------------
# Consumed by add_subdirectory, like treemapper_ros does with the core: found through the REAL path of this package
# (it is usually symlinked into a catkin workspace's src/). nav/corridor_nav adds the core itself; its own gtest
# suite is switched off here (catkin_add_gtest brings the gtest targets, scripts/run_tests.sh runs the core suites).
get_filename_component(_cn_pkg_real "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(CN_NAV_DIR "${_cn_pkg_real}/../../nav/corridor_nav" CACHE PATH "Path to the navigator library (nav/corridor_nav)")
get_filename_component(CN_NAV_DIR "${CN_NAV_DIR}" REALPATH)
if(NOT EXISTS "${CN_NAV_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "corridor_nav not found at ${CN_NAV_DIR}; pass -DCN_NAV_DIR=<repo>/nav/corridor_nav")
endif()
set(CORRIDOR_NAV_BUILD_TESTS OFF CACHE BOOL "nav suites are run by scripts/run_tests.sh, not by catkin" FORCE)
set(_cn_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory("${CN_NAV_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/corridor_nav")
set(BUILD_SHARED_LIBS ${_cn_shared})
message(STATUS "corridor_nav_ros: navigator from ${CN_NAV_DIR}")

set(CN_WARN -Wall -Wextra -Wpedantic)

add_library(corridor_nav_ros_support src/conversions.cpp src/field_file.cpp)
target_include_directories(corridor_nav_ros_support PUBLIC include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(corridor_nav_ros_support PUBLIC corridor_nav_core ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_compile_options(corridor_nav_ros_support PRIVATE ${CN_WARN})
add_dependencies(corridor_nav_ros_support ${catkin_EXPORTED_TARGETS})

add_executable(corridor_nav_node src/corridor_nav_node.cpp)
target_link_libraries(corridor_nav_node PRIVATE corridor_nav_ros_support ${catkin_LIBRARIES})
target_compile_options(corridor_nav_node PRIVATE ${CN_WARN})

install(TARGETS corridor_nav_node corridor_nav_ros_support
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_corridor_nav_ros test/test_conversions.cpp test/test_field_file.cpp)
  target_link_libraries(test_corridor_nav_ros corridor_nav_ros_support ${catkin_LIBRARIES})
  add_dependencies(test_corridor_nav_ros corridor_nav_ros_support ${catkin_EXPORTED_TARGETS})
  target_compile_options(test_corridor_nav_ros PRIVATE ${CN_WARN})
  target_compile_definitions(test_corridor_nav_ros PRIVATE CN_ROS_PKG_DIR="${_cn_pkg_real}")
endif()
