cmake_minimum_required(VERSION 3.0.2)
project(st_arm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES st_arm_control
  CATKIN_DEPENDS 
    cmake_modules 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    std_msgs 
  DEPENDS 
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${RBDL_INCLUDE_DIR}
)

add_executable(
  st_arm_control
  src/main.cpp 
  src/dynamics.cpp
  src/callback.cpp
)
add_dependencies(st_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(st_arm_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${RBDL_LIBRARY})

install(TARGETS st_arm_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
