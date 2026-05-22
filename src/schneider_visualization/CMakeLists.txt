cmake_minimum_required(VERSION 3.0.2)
project(schneider_visualization)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf2_ros visualization_msgs)
catkin_package()
catkin_install_python(PROGRAMS src/visualization_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
