cmake_minimum_required(VERSION 3.0.2)
project(st_arm_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    gazebo_ros
    gazebo_ros_control
    controller_manager
    xacro
    urdf
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    gazebo_ros
    gazebo_ros_control
    controller_manager
    xacro
    urdf
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)