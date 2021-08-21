# rm_shooter_controller

## 1. Overview

The Controller is RoboMaster robot shooter controller, it accepts instructions from command topic and sets the order to the friction wheel motor

**Keywords:** shooter

#### License

The source code is released under a [BSD 3-Clause license]().

**Author: DynamicX**

**Affiliation: DynamicX**

**Maintainer: DynamicX**

The rm_shooter_controller package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### 1.1. Hardware interface type

The controller works with friction wheel joints through a **effort** interface.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

## 2. Installation

### 2.1. Installation from Packages

To install all packages from the this repository as Debian packages use

```plaintext
sudo apt-get install ros-noetic-...
```

or better use `rosdep`:

```
sudo rosdep install --from-paths src 
```

### 2.2. Building from Source

#### 2.2.1 Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org/) (middleware for robotics),
- rm_description
- roscpp
- roslint
- rm_msgs
- rm_common
- pluginlib
- controller_interface
- hardware_interface
- controller_interface
- forward_command_controller
- realtime_tools
- control_toolbox
- effort_controllers
- dynamic_reconfigure

#### 2.2.2 Building

+ Build this package with catkin build. Clone the latest version from this repository into your catkin workspace.

## 3. Usage

Run the controller with mon launch:

```
mon launch rm_shooter_controller load_controllers.launch
```

## 4. Cfg

+ **shooter.cfg:** Add parameters related to friction wheel speed corresponding to each bullet speed and trigger block detection parameters

## 5. Launch files

- **load_controller.launch:** Load the parameters in config files and load the shooter controller.

## 6. ROS API

#### 6.1. Subscribed Topics

* `command`(rm_msgs/ShootCmd)

  Commands of controller state, bullet speed, frequency of shooting, and time stamp.

#### 6.2. Parameters

* `block_effort`, `block_speed`, `block_duration` (`double`, default: 0)

  When the torque of the plucking motor is greater than block_effort (in N·m), and the angular velocity is less than block_speed (in rad/s), it will be regarded as jamming if it continues for block_duration (in s).

* `block_overtime` (`double`, default: 0)

  If the time to enter block state exceeds block_overtime (in s), it will be judged as timeout and exit block state.

* `anti_block_angle` (`double`, default: 0)

  If enter block state, the friction wheel will reverse anti_block_angle (in rad/s) to try to get rid of the structing.

* `anti_block_threshold` (`double`, default: 0)

  If the anti angle of the friction wheel exceeds anti_block_threshold, it means that trigger reverse success.

* `qd_10`, `qd_15`, `qd_18`, `qd_30`(`double`)

  It means friction wheel's angular velocity, the number of it's name expresses different bullet speeds (in m/s).

### 7. Controller configuration examples

#### 7.1. Minimal description

```
shooter_controller:
    type: rm_shooter_controllers/Controller
    publish_rate: 50
    friction_left:
      joint: "left_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    friction_right:
      joint: "right_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    trigger:
      joint: "trigger_joint"
      pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
```

#### 7.2. Complete description

```
shooter_controller:
    type: rm_shooter_controllers/Controller
    publish_rate: 50
    friction_left:
      joint: "left_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    friction_right:
      joint: "right_friction_wheel_joint"
      pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
    trigger:
      joint: "trigger_joint"
      pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
    push_per_rotation: 8
    push_qd_threshold: 0.90
    block_effort: 0.95
    block_duration: 0.05
    block_overtime: 0.5
    anti_block_angle: 0.2
    anti_block_threshold: 0.1
    qd_15: 460.0
    qd_18: 515.0
    qd_30: 740.0
```

## 8. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues) .