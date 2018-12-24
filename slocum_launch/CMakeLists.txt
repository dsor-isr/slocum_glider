cmake_minimum_required(VERSION 2.8.3)
project(slocum_launch)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/serial_backseat_interface.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
