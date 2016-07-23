cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_moveit)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS include)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(quadrotor_moveit src/main.cpp src/quadrotor.cpp)
target_link_libraries(quadrotor_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS quadrotor_moveit DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
