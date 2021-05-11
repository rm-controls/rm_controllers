cmake_minimum_required(VERSION 3.10)
project(rm_chassis_controllers)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        rostest

        rm_msgs
        rm_common
        robot_state_controller

        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
        effort_controllers
        tf2
        tf2_geometry_msgs
        angles
        )

find_package(Eigen3 REQUIRED)

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
        ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        roslint
        rostest

        rm_msgs
        rm_common
        robot_state_controller


        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
        effort_controllers
        tf2
        tf2_geometry_msgs
        angles
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

## Declare cpp executables
add_library(${PROJECT_NAME}
        src/chassis_base.cpp
        src/mecanum.cpp
        src/sentry.cpp
        src/balance.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
#    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#    DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp"
#)
#
## Mark other files for installation
#install(
#    DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
    ## Add gtest based cpp test target and link libraries
    add_rostest_gtest(standard_chassis_test
            test/test_standard_chassis.test
            test/test_standard_chassis.cpp)
    target_link_libraries(standard_chassis_test ${catkin_LIBRARIES})


endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
