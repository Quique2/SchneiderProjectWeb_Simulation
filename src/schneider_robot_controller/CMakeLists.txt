cmake_minimum_required(VERSION 3.0.2)
project(schneider_robot_controller)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)
catkin_package()
catkin_install_python(PROGRAMS src/robot_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
