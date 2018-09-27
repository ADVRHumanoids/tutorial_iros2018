project(cartesio)
cmake_minimum_required(VERSION 3.0.0)

find_package(cartesian_interface REQUIRED)

include_directories(controllers/ 
                    ../opensot/include/ 
                    ${cartesian_interface_INCLUDE_DIRS})

add_library(CartesianSimpleIk controllers/SimpleIkController.cpp)
target_link_libraries(CartesianSimpleIk ${cartesian_interface_LIBRARIES} IKProblem)

install(TARGETS CartesianSimpleIk CartesianSimpleIk
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)