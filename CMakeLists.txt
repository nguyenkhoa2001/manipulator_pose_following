cmake_minimum_required(VERSION 2.8.3)
project(manipulator_pose_following)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_manipulation
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  message_generation
  cmake_modules
  actionlib
  control_msgs
  sensor_msgs
  trajectory_msgs
)

find_package (Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 DeltaPoseRPY.msg
 PlannedPath.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  InitPoint.srv
  MoveHome.srv
  MoveToRPY.srv
  MoveToQuat.srv
  ReplyInt.srv
  SetVelocity.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES manipulator_pose_following
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_manipulation
    message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header filesmanipulator_pose_following
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/manipulator_pose_following.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(test_traject src/test_traject_node.cpp)
add_executable(moveit_interface src/moveit_interface_node.cpp)
add_executable(pose_following src/pose_following_node.cpp)
add_executable(smooth_jogging src/smooth_jogging_node.cpp)
add_executable(kb_jogging src/kb_jogging_node.cpp)

add_executable(test_keyboard_node src/test_keyboard.cpp)
add_executable(test_service src/test_service.cpp)
add_executable(user_pose_testing_node src/user_pose_testing.cpp)
add_executable(control_with_leap_node src/control_with_leap.cpp)
add_executable(check_kinematic_node src/check_kinematic_node.cpp)
add_executable(point_traj_following src/point_traj_following.cpp)
add_executable(read_planned_file src/read_file)
add_executable(send_trajectory_2actionlib src/send_trajectory_2actionlib.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(user_pose_testing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_with_leap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(send_trajectory_2actionlib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(point_traj_following ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(read_planned_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(test_traject
  ${catkin_LIBRARIES}
)
target_link_libraries(moveit_interface
  ${catkin_LIBRARIES}
)
target_link_libraries(pose_following
  ${catkin_LIBRARIES}
)
target_link_libraries(smooth_jogging
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(kb_jogging
  ${catkin_LIBRARIES}
  ncurses
)

target_link_libraries(test_keyboard_node
  ${catkin_LIBRARIES}
)
target_link_libraries(test_service
  ${catkin_LIBRARIES}
)
target_link_libraries(user_pose_testing_node
  ${catkin_LIBRARIES}
)
target_link_libraries(control_with_leap_node
  ${catkin_LIBRARIES}
)
target_link_libraries(point_traj_following
  ${catkin_LIBRARIES}
)
target_link_libraries(check_kinematic_node
  ${catkin_LIBRARIES}
)
target_link_libraries(read_planned_file
  ${catkin_LIBRARIES}
)
target_link_libraries(send_trajectory_2actionlib
  ${catkin_LIBRARIES}
)
