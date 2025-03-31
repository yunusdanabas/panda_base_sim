cmake_minimum_required(VERSION 2.8.3)
project(panda_base_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  xacro
  robot_state_publisher
  joint_state_publisher
  controller_manager
  rviz
  franka_gazebo
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros xacro robot_state_publisher joint_state_publisher controller_manager rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

