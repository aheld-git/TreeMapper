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

# --- Mapper registry + candidate mapping systems (see PROGRESS.md, "How to add a candidate") ----------
# The registry (src/mapper_registry.cpp) and every candidate are compiled into the OBJECT library
# treemapper_registry. Executables take its object files directly via tm_link_registry(): a static
# library would not do, because the registry reaches candidates through weak symbols and the linker
# never pulls archive members in for a weak reference.
#
# A candidate owns src/candidates/<name>/*.cpp, include/treemapper/candidates/<name>/ and optionally
# tests/candidates/<name>/*.cpp plus src/candidates/<name>/candidate.cmake (extra find_package / link
# lines; it runs after treemapper_core and treemapper_registry exist: link libraries to treemapper_core
# with PUBLIC). All of it is found by glob: no edit here. The only shared edit is one TM_REGISTER_CANDIDATE
# line in src/mapper_registry.cpp.
# -DTM_CANDIDATES="cand_a;cand_b" builds only those (default "all"); a candidate that is not built simply
# drops out of the registry, so a half-written candidate cannot break another's build.
set(TM_CANDIDATES "all" CACHE STRING "Candidate directories to build: 'all' or a ;-list of names")
add_library(treemapper_registry OBJECT src/mapper_registry.cpp)
target_include_directories(treemapper_registry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(treemapper_registry PUBLIC treemapper_core)
target_compile_options(treemapper_registry PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(treemapper_registry PROPERTIES POSITION_INDEPENDENT_CODE ON)

file(GLOB TM_CANDIDATE_DIRS CONFIGURE_DEPENDS LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/src/candidates/*)
set(TM_CANDIDATE_TEST_SOURCES "")
set(TM_CANDIDATE_NAMES "")
foreach(dir ${TM_CANDIDATE_DIRS})
  if(IS_DIRECTORY ${dir})
    get_filename_component(cand ${dir} NAME)
    if(TM_CANDIDATES STREQUAL "all" OR cand IN_LIST TM_CANDIDATES)
      file(GLOB cand_src CONFIGURE_DEPENDS ${dir}/*.cpp)
      file(GLOB cand_test CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/candidates/${cand}/*.cpp)
      target_sources(treemapper_registry PRIVATE ${cand_src})
      list(APPEND TM_CANDIDATE_TEST_SOURCES ${cand_test})
      list(APPEND TM_CANDIDATE_NAMES ${cand})
      set(TM_CANDIDATE_DIR ${dir})
      if(EXISTS ${dir}/candidate.cmake)
        include(${dir}/candidate.cmake)
      endif()
    endif()
  endif()
endforeach()
message(STATUS "treemapper candidates built: ${TM_CANDIDATE_NAMES}")

# Links `target` against the core and the registry (with all enabled candidates).
function(tm_link_registry target)
  target_sources(${target} PRIVATE $<TARGET_OBJECTS:treemapper_registry>)
  target_link_libraries(${target} PRIVATE treemapper_core)
endfunction()

# Offline tooling: frame-file reader + replay runner (not needed by the ROS nodes).
add_library(treemapper_io src/frame_io.cpp)
target_link_libraries(treemapper_io PUBLIC treemapper_core)
target_compile_options(treemapper_io PRIVATE -Wall -Wextra -Wpedantic)
add_executable(treemapper_replay apps/replay_main.cpp)
target_link_libraries(treemapper_replay PRIVATE treemapper_io)
tm_link_registry(treemapper_replay)
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
  add_executable(test_core tests/test_tree_map.cpp tests/test_simple_mapper.cpp tests/test_mapper_registry.cpp
                 ${TM_CANDIDATE_TEST_SOURCES})
  target_link_libraries(test_core PRIVATE treemapper_core GTest::gtest_main)
  tm_link_registry(test_core)
  add_test(NAME test_core COMMAND test_core)
endif()

install(TARGETS treemapper_core treemapper_io ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
