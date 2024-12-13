cmake_minimum_required(VERSION 3.0.2)
project(wild_visual_navigation_real_world)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  wild_visual_navigation_ros
)

catkin_package(
  CATKIN_DEPENDS
)

message(${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

############
## Python ##
############
catkin_install_python(PROGRAMS
                               scripts/scout_state_converter_node.py
                               scripts/robot_convert.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})