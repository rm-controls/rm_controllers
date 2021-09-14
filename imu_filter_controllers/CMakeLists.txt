cmake_minimum_required(VERSION 3.0.2)
project(imu_filter_controllers)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        imu_complementary_filter
        rm_common
        hardware_interface
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        roslint

        imu_complementary_filter
        rm_common
        hardware_interface
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
        src/complementary_controller.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )


roslint_cpp()