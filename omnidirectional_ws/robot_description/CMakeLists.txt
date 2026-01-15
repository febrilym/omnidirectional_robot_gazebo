cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xacro
  urdf
  gazebo_ros
  gazebo_ros_control
  controller_manager
  joint_state_publisher
  robot_state_publisher
)

# packages
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    xacro
    urdf
    gazebo_ros
    gazebo_ros_control
    controller_manager
    joint_state_publisher
    robot_state_publisher
)

# build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install
install(DIRECTORY launch config meshes robots worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
