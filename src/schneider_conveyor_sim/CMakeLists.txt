cmake_minimum_required(VERSION 3.0.2)
project(schneider_conveyor_sim)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()
catkin_install_python(PROGRAMS src/conveyor_sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
