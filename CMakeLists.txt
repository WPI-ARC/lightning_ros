cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(collision_checker_l src/tools/collision_checker.cpp
                                         src/tools/collision_utils.cpp)
rosbuild_add_library(planner_stoppable_l src/planner_stoppable/ompl_ros_stoppable.cpp  
                                         src/planner_stoppable/ompl_ros_stoppable_joint_planner.cpp  
                                         src/planner_stoppable/ompl_ros_stoppable_planning_group.cpp)
rosbuild_link_boost(planner_stoppable_l thread)
rosbuild_add_executable(planner_stoppable_main src/planner_stoppable/planner_stoppable_main.cpp)
rosbuild_add_executable(collision_check_service src/tools/collision_check_service.cpp)
rosbuild_add_executable(path_shortcutter src/tools/path_shortcutter.cpp)
target_link_libraries(planner_stoppable_main planner_stoppable_l)
target_link_libraries(collision_check_service collision_checker_l)
target_link_libraries(path_shortcutter collision_checker_l)

