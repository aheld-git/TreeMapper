cmake_minimum_required(VERSION 3.0.2)
project(treemapper_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros topic_tools treemapper_msgs)

catkin_package(CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros
               topic_tools treemapper_msgs message_runtime)

# --- the ROS-free core (core/ of this repository), built in-tree --------------------------------------
# The core is consumed by add_subdirectory, not as an installed package: the mapper registry reaches the
# candidates through weak symbols, so every executable must take the registry's object files directly via
# tm_link_registry() (core/CMakeLists.txt). TM_CORE_DIR defaults to <repo>/core, found through the REAL path
# of this package (it is usually symlinked into a catkin workspace's src/).
get_filename_component(_tm_pkg_real "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(TM_CORE_DIR "${_tm_pkg_real}/../../core" CACHE PATH "Path to the treemapper core (core/ of the repo)")
get_filename_component(TM_CORE_DIR "${TM_CORE_DIR}" REALPATH)
if(NOT EXISTS "${TM_CORE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "treemapper core not found at ${TM_CORE_DIR}; pass -DTM_CORE_DIR=<repo>/core")
endif()
set(TREEMAPPER_BUILD_TESTS OFF CACHE BOOL "core unit tests are run by scripts/run_tests.sh, not by catkin" FORCE)
set(_tm_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)  # core as a static, PIC library linked into the node: nothing extra to install
add_subdirectory("${TM_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/treemapper_core")
set(BUILD_SHARED_LIBS ${_tm_shared})
message(STATUS "treemapper_ros: core from ${TM_CORE_DIR}")

set(TM_WARN -Wall -Wextra)

add_library(treemapper_ros_support src/input_sync.cpp src/node_support.cpp src/conversions.cpp)
target_include_directories(treemapper_ros_support PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(treemapper_ros_support PUBLIC treemapper_core ${catkin_LIBRARIES})
target_compile_options(treemapper_ros_support PRIVATE ${TM_WARN})
add_dependencies(treemapper_ros_support ${catkin_EXPORTED_TARGETS})

add_executable(treemapper_node src/treemapper_node.cpp)
target_link_libraries(treemapper_node PRIVATE treemapper_ros_support ${catkin_LIBRARIES})
tm_link_registry(treemapper_node)
target_compile_options(treemapper_node PRIVATE ${TM_WARN})

install(TARGETS treemapper_node treemapper_ros_support
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/nav_emulator.py scripts/treemapper_bag_to_jsonl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_treemapper_ros test/test_input_sync.cpp test/test_conversions.cpp test/test_config_agreement.cpp)
  find_package(yaml-cpp REQUIRED)
  target_include_directories(test_treemapper_ros PRIVATE ${YAML_CPP_INCLUDE_DIR})
  # = tm_link_registry(), spelled out: catkin_add_gtest already used the plain target_link_libraries signature,
  # which cannot be mixed with the keyword form tm_link_registry() uses.
  target_sources(test_treemapper_ros PRIVATE $<TARGET_OBJECTS:treemapper_registry>)
  target_link_libraries(test_treemapper_ros treemapper_ros_support treemapper_core ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
  target_compile_options(test_treemapper_ros PRIVATE ${TM_WARN})
  target_compile_definitions(test_treemapper_ros PRIVATE TM_ROS_PKG_DIR="${_tm_pkg_real}" TM_CORE_DIR="${TM_CORE_DIR}")
  find_package(rostest REQUIRED)
  add_rostest(test/replay_smoke.test)
endif()
