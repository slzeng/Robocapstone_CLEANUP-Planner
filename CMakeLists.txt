cmake_minimum_required(VERSION 2.8.3)
project(planner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf actionlib_msgs actionlib)

## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

add_action_files(DIRECTORY action FILES gen_traj.action)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(planner_node src/planner.cpp)

target_link_libraries(planner_node ${catkin_LIBRARIES})

add_dependencies(
  planner_node
  ${planner_EXPORTED_TARGETS}
)