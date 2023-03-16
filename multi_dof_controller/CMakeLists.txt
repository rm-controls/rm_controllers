cmake_minimum_required(VERSION 3.10)
project(multi_dof_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        geometry_msgs

        rm_common

        controller_interface
        effort_controllers
        )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        geometry_msgs

        rm_common

        effort_controllers
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
## Declare a cpp library
add_library(${PROJECT_NAME} src/multi_dof_controller.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(TARGETS multi_dof_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES multi_dof_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


#############
## Testing ##
#############
