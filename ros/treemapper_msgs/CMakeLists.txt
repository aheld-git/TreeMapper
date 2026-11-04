cmake_minimum_required(VERSION 3.0.2)
project(treemapper_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(FILES
  TreeEntry.msg
  TreeMap.msg
  Detection2D.msg
  Detection2DArray.msg
  DronePose.msg
  MissionPhase.msg
  Health.msg
  DropCounters.msg
  StageTiming.msg
  DisparityGrid.msg
)
add_service_files(FILES
  GetTrees.srv
  SetTreeSprayed.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
