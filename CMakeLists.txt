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

        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
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

        pluginlib
        hardware_interface
        controller_interface
        forward_command_controller
        realtime_tools
        control_toolbox
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
        src/standard.cpp
        src/sentry.cpp
        #        src/balance.cpp
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

    add_rostest_gtest(scopperil_test
            test/test_scopperil.test
            test/test_scopperil.cpp)
    target_link_libraries(scopperil_test ${catkin_LIBRARIES})

    add_rostest_gtest(default_cmd_publisher_test
            test/test_default_cmd_publisher.test
            test/test_default_cmd_publisher.cpp)
    target_link_libraries(default_cmd_publisher_test ${catkin_LIBRARIES})

    add_rostest_gtest(wrong_joint_name_test
            test/test_wrong_joint_name.test
            test/test_wrong_joint_name.cpp)
    target_link_libraries(wrong_joint_name_test ${catkin_LIBRARIES})

    add_rostest_gtest(default_odom_frame_test
            test/test_default_odom_frame.test
            test/test_default_odom_frame.cpp)
    target_link_libraries(default_odom_frame_test ${catkin_LIBRARIES})

    add_rostest_gtest(odom_tf_test
            test/test_odom_tf.test
            test/test_odom_tf.cpp)
    target_link_libraries(odom_tf_test ${catkin_LIBRARIES})

    add_rostest_gtest(timeout_test
            test/test_timeout.test
            test/test_timeout.cpp)
    target_link_libraries(timeout_test ${catkin_LIBRARIES})

    add_rostest_gtest(linear_x_accel_limit_test
            test/test_linear_x_accel_limit.test
            test/test_linear_x_accel_limit.cpp)
    target_link_libraries(linear_x_accel_limit_test ${catkin_LIBRARIES})

    add_rostest_gtest(linear_y_accel_limit_test
            test/test_linear_y_accel_limit.test
            test/test_linear_y_accel_limit.cpp)
    target_link_libraries(linear_y_accel_limit_test ${catkin_LIBRARIES})

    add_rostest_gtest(angular_z_accel_limit_test
            test/test_angular_z_accel_limit.test
            test/test_angular_z_accel_limit.cpp)
    target_link_libraries(angular_z_accel_limit_test ${catkin_LIBRARIES})

    add_rostest_gtest(iKine_test
            test/test_iKine.test
            test/test_iKine.cpp)
    target_link_libraries(iKine_test ${catkin_LIBRARIES})

endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
