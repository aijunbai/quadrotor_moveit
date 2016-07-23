# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(action_controller)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS genmsg actionlib actionlib_msgs roscpp roslib std_msgs geometry_msgs moveit_msgs message_generation)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#add_message_files(
#  FILES
#  MultiDofFollowJointTrajectoryActionFeedback.msg  MultiDofFollowJointTrajectoryActionResult.msg  MultiDofFollowJointTrajectoryResult.msg
#  MultiDofFollowJointTrajectoryActionGoal.msg      MultiDofFollowJointTrajectoryFeedback.msg
#  MultiDofFollowJointTrajectoryAction.msg          MultiDofFollowJointTrajectoryGoal.msg
#)
add_action_files(DIRECTORY action FILES MultiDofFollowJointTrajectory.action)
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs std_msgs geometry_msgs)

catkin_package(
    DEPENDS actionlib roscpp roslib std_msgs geometry_msgs moveit_msgs actionlib_msgs trajectory_msgs
    CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

find_package (Threads)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(${Boost_INCLUDE_DIRS})
add_executable(actionController src/actionController.cpp)
add_dependencies(actionController ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(actionController ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

