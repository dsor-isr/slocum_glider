cmake_minimum_required(VERSION 2.8.3)
project(slocum_glider_extctl)

find_package(
  catkin REQUIRED
  COMPONENTS
  roslint
)

catkin_package()

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--ignore=W503,E241,E226,E123")
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SRCS *.py)
roslint_python(${${PROJECT_NAME}_PYTHON_SRCS})
