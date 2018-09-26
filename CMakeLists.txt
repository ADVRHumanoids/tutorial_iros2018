cmake_minimum_required(VERSION 2.8)
project(tutorial_iros2018)

find_package(catkin REQUIRED)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
#  CATKIN_DEPENDS 
#  DEPENDS 
)

add_subdirectory(plugins/)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY cartesio/problems/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cartesio/problems
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
)


