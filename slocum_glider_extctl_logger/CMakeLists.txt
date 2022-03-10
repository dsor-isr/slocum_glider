cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_extctl_logger)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()

catkin_python_setup()

roslint_python(scripts/slocum_glider_extctl_logger)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/extctl_logger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/logger.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
