cmake_minimum_required(VERSION 3.10)
project(rm_chassis_controllers)

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
    rostest

    rm_msgs
    rm_common
    rm_base

    tf2
    tf2_geometry_msgs

    control_toolbox
    angles
    controller_interface
    forward_command_controller
    hardware_interface
    pluginlib
    realtime_tools
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
    CATKIN_DEPENDS
    roscpp
    roslint
    rostest

    rm_msgs
    rm_common
    rm_base

    tf2
    tf2_geometry_msgs

    control_toolbox
    angles
    controller_interface
    forward_command_controller
    hardware_interface
    pluginlib
    realtime_tools

    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_library(${PROJECT_NAME}
    src/standard.cpp
    src/sentry.cpp
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

  add_rostest_gtest(nan_cmd_vel_test
          test/test_NAN_cmd.test
          test/test_NAN_cmd.cpp)
  target_link_libraries(nan_cmd_vel_test ${catkin_LIBRARIES})

  add_rostest_gtest(linear_x_direction_jerk_limit_test
          test/test_linear_x_jerk_limit.test
          test/test_linear_x_jerk_limit.cpp)
  target_link_libraries(linear_x_direction_jerk_limit_test ${catkin_LIBRARIES})

  add_rostest_gtest(linear_y_direction_jerk_limit_test
          test/test_linear_y_jerk_limit.test
          test/test_linear_y_jerk_limit.cpp)
  target_link_libraries(linear_y_direction_jerk_limit_test ${catkin_LIBRARIES})

  add_rostest_gtest(linear_x_velocity_limit_test
          test/test_linear_x_velocity_limit.test
          test/test_linear_x_velocity_limit.cpp)
  target_link_libraries(linear_x_velocity_limit_test ${catkin_LIBRARIES})

  add_rostest_gtest(linear_y_velocity_limit_test
          test/test_linear_y_velocity_limit.test
          test/test_linear_y_velocity_limit.cpp)
  target_link_libraries(linear_y_velocity_limit_test ${catkin_LIBRARIES})

  add_rostest_gtest(angular_z_velocity_limit_test
          test/test_angular_z_velocity_limit.test
          test/test_angular_z_velocity_limit.cpp)
  target_link_libraries(angular_z_velocity_limit_test ${catkin_LIBRARIES})

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

endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
