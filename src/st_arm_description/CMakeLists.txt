cmake_minimum_required(VERSION 2.8.3)

project(st_arm_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/update_gazebo_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)