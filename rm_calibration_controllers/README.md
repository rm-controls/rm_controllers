# rm_calibration_controllers

## 1. Overview

Since the zero point of some actuators will change after power off, rm_calibration_controller will move at a certain speed after it is started until it reaches the mechanical limit, and the motor position will be reset to zero.

***Keywords***: calibration, ROS, position.

### License
The source code is released under a [ BSD 3-Clause license](http://192.168.0.100:7070/dynamicx/rm_gimbal_controllers/-/blob/master/LICENSE).
#### Author: DynamicX
#### Affiliation: DynamicX

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

## 2. Installation

#### 2.1. Installation from Packages


To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-calibration-controllers
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

#### 2.2. Building from Source

##### 2.2.1. Dependencies
* roscpp
* roslint
* rm_msgs
* rm_common
* pluginlib
* hardware_interface
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


## 3. ROS API

#### 3.1 Service
* **`is_calibrated_srv_`** ([control_msgs/QueryCalibrationState](http://docs.ros.org/en/api/control_msgs/html/srv/QueryCalibrationState.html))

	 Returns information about whether target controller has been calibrated.


#### 3.2 Parameters
* **`search_velocity`** (double)

	The actuator velocity of calibrating.

* **`threshold`** (double)

	This is velocity threshold. When the real time velocity of target actuator lower than threshold, and last for a while, it can be considered the state: CALIBRATED.


## 4. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
