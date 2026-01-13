cmake_minimum_required(VERSION 3.0.2)
project(soccer_robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/soccer_robot_control.py
  scripts/simple_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/omni_drive_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)