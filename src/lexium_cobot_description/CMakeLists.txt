cmake_minimum_required(VERSION 3.0.2)
project(lexium_cobot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf meshes config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
