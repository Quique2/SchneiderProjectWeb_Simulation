cmake_minimum_required(VERSION 3.0.2)
project(lexium_cobot_urdfbueno)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf meshes launch rviz scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
