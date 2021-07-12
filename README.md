# rm_shooter_controller

## Overview

The Controller is RoboMaster robot shooter controller. It is used for reading joint sensor data and sending command to motors. Besides , It will calls control loop (update method) periodically at a set frequency .

**Keywords:** shooter

#### License

The source code is released under a [BSD 3-Clause license]().

**Author: Unknown  **

**Affiliation: DynamicX Maintainer: none **

The rm_shooter_controller package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

```plaintext
sudo apt-get install ros-noetic-...
```

or better use `rosdep`:

```
sudo rosdep install --from-paths src 
```

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org/) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- sensor_msgs
- rm_common
- realtime_tools
- tf2
- tf2_geometry_msgs
- angles

#### Building

+ Build this package with catkin build. Clone the latest version from this repository into your catkin workspace.

## Usage

Run the controller with mon launch:

```
mon launch rm_shooter_controller load_controllers.launch
```

## Cfg

+ **shooter.cfg:** The following shooter.cfg is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface

## Config

Config file config

+ **hero.yaml:** it loads some controllers and the parameters used for hero robot into the parameter server

+ **sentry.yaml:** it loads some controllers and the parameters used for sentry robot into the parameter server

+ **standard3.yaml:** it loads some controllers and the parameters used for standard3 robot into the parameter server

+ **standard4.yaml:** it loads some controllers and the parameters used for standard4 robot into the parameter server

+ **standard5.yaml:** it loads some controllers and the parameters used for standard5 robot into the parameter server

## Launch files

- **load_controller.launch:** launch the codes of rm_shooter_controller

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues) .
