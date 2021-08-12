cmake_minimum_required(VERSION 3.10)
project(robot_state_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_common
        hardware_interface
        controller_interface
        urdf
        tf2_ros
        tf2_kdl
        kdl_parser
        realtime_tools
        pluginlib
        )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint

        rm_common
        hardware_interface
        controller_interface
        urdf
        tf2_ros
        kdl_parser
        realtime_tools
        pluginlib
        LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
install(
        FILES robot_state_controller_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation
#install(
#    DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
