cmake_minimum_required(VERSION 3.0.2)
project(schneider_cell_description)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/safe_joint_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch meshes config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
