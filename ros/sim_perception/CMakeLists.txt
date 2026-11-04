cmake_minimum_required(VERSION 3.0.2)
project(sim_perception)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs message_filters treemapper_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy sensor_msgs message_filters treemapper_msgs)

catkin_install_python(PROGRAMS scripts/stereo_sgbm_node.py scripts/detector_node.py scripts/disparity_grid_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Unit tests are plain pytest and need no ROS graph: scripts/run_tests.sh runs
#   PYTHONPATH=ros/sim_perception/src python3 -m pytest -q ros/sim_perception/test
