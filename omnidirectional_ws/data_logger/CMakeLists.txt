cmake_minimum_required(VERSION 2.8.3)
project(data_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  gazebo_msgs
)

catkin_package()

# install
install(PROGRAMS
  scripts/soccer_robot_logger.py
  scripts/odometry_logger.py
  scripts/bag_to_csv.py
  scripts/visualize_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)