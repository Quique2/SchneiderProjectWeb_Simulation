cmake_minimum_required(VERSION 3.0.2)
project(schneider_object_manager)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf2_ros visualization_msgs)
catkin_package()
catkin_install_python(PROGRAMS src/object_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
