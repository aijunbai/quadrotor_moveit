cmake_minimum_required(VERSION 2.8.3)
project(moveit_controller)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} "/home/baj/Dropbox/Workspace/catkin-workspace/src/autonomous-flight-ros/action_controller/msg_gen/cpp/include")
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  DEPENDS
    moveit_core
)

include_directories(include)

add_library(moveit_controller src/moveit_controller.cpp)
target_link_libraries(moveit_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_controller LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_controller_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
