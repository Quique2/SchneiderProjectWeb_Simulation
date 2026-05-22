cmake_minimum_required(VERSION 3.0.2)
project(schneider_gripper_sim)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs tf2_ros)
catkin_package()
catkin_install_python(PROGRAMS src/gripper_sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
