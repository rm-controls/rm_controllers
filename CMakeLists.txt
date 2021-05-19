cmake_minimum_required(VERSION 3.0.2)
project(rm_orientation_controllers)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    controller_interface
    hardware_interface
    realtime_tools
    roscpp
    sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

