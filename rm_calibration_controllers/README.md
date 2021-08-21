# rm_calibration_controllers

## 1. Overview

The Controller is RoboMaster robot calibration controller. It is used for calibrating position such as joint, sentry.

***Keywords***: calibration, ROS, position.

### License
The source code is released under a [ BSD 3-Clause license](http://192.168.0.100:7070/dynamicx/rm_gimbal_controllers/-/blob/master/LICENSE).
#### Author: DynamicX
#### Affiliation: DynamicX

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

## 2. Installation

#### 2.1. Installation from Packages
    sudo apt-get install ros-noetic-...
Or better, use `rosdep`:

    sudo rosdep install --from-paths src

#### 2.2. Building from Source
##### 2.2.1. Dependencies
* roscpp
* roslint
*
* rm_msgs
* rm_common
* pluginlib
*  hardware_interface
* controller_interface
* realtime_tools
* effort_controllers
* control_msgs


#### 2.2.2. Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/rm-controls/rm_controllers.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## 3. Usage

* Run the controller with mon launch:

      mon launch rm_bringup engineer.launch

## 4. Config

* ***engineer.yaml***: Load parameters in the config, and load mast, card, stone_platform, yaw, pitch and joint_x calibration controller.
* ***hero.yaml***: Load parameters in the config, and load trigger calibration controller.
* ***sentry.yaml***: Load parameters in the config, and load trigger, pitch and yaw calibration controller.
* ***standard3.yaml***: Load parameters in the config, and load trigger and cover calibration controller.
* ***standard4.yaml***: Load parameters in the config, and load trigger and cover calibration controller.
* ***standard5.yaml***: Load parameters in the config, and load trigger and cover calibration controller.

## 5. ROS API


### 5.1. Services
* `is_calibrated_srv_` (struct)

  Used to judge whether the calibration process has been finished.


### 5.2. Parameters
* `search_velocity`(double)

  When the velocity of calibrated target is lower than search velocity, it can be considered as calibrating successful.

* `threshold`(double)

  When the calibrating successful time more than threshold, it can be considered that target has been calibrated.

* `pid`(double)

  The pid of calibrate controller.


## 6. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
