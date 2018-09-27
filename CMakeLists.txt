cmake_minimum_required(VERSION 2.8)
project(tutorial_iros2018)

find_package(catkin REQUIRED urdf)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
#  CATKIN_DEPENDS 
#  DEPENDS 
)

add_compile_options("-std=c++11")

add_executable(joint_map_generator src/generate_joint_map.cpp)
target_link_libraries(joint_map_generator ${catkin_LIBRARIES})

add_subdirectory(plugins/)
add_subdirectory(opensot/)
add_subdirectory(cartesio/)

install(TARGETS joint_map_generator joint_map_generator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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


