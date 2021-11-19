cmake_minimum_required(VERSION 3.0.2)
project(imu_filter_controllers)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rm_msgs
        rm_common

        pluginlib
        imu_complementary_filter
        imu_filter_madgwick
        hardware_interface
        controller_interface
        realtime_tools
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        roslint

        rm_msgs
        rm_common

        pluginlib
        imu_complementary_filter
        imu_filter_madgwick
        hardware_interface
        controller_interface
        realtime_tools
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
        src/imu_filter_base.cpp
        src/complementary_controller.cpp
        src/madgwick_controller.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )


roslint_cpp()
