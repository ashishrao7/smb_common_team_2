cmake_minimum_required(VERSION 3.0.2)
project(launch_scripts)
find_package(catkin REQUIRED COMPONENTS 
roslaunch
)


catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

roslaunch_add_file_check(launch)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)