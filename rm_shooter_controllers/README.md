# rm_shooter_controller

## 1. Overview

The controller is RoboMaster robot shooter controller. It is used for reading joint sensor data and sending command to motors. Besides , It will calls control loop (update method) periodically at a set frequency .

**Keywords:** shooter

#### License

The source code is released under a [BSD 3-Clause license]().

**Author: Unknown  **

**Affiliation: DynamicX Maintainer: none **

The rm_shooter_controller package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### 1.1. Hardware interface type

The controller works with friction wheel joints through a **velocity** interface.

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

#### 2.2.2 Building

+ Build this package with catkin build. Clone the latest version from this repository into your catkin workspace.

## 3. Usage

Run the controller with mon launch:

```
mon launch rm_shooter_controller load_controllers.launch
```

## 4. Cfg

+ **shooter.cfg:** Add parameters related to friction wheel speed and trigger block detection corresponding to each bullet speed

## 5. Launch files

- **load_controller.launch:** Load the parameters in config files and load the shooter controller.

## 6. ROS API

### 6.1. Description

The controller main input is command topic in the namespace of the controller.

#### 6.2. Subscribed Topics

`command`

- Commands of controller status, bullet speed, frequency of shooting, hatch cover status and time stamp

#### 6.3. Parameters

`block_effort` (`double`, default: 0)

+ Upper limit moment of trigger block effort, It‘s minimum value is 0.0 and its maximum value is  10.

`block_speed` (`double`, default: 0)

- Lowest limit speed of speed, If the speed is lower than this speed, it would be judged as blocked.

`block_duration` (`double`, default: 0)

- The jam duration of blocked ammunition. If the jam time is over this duration, it would be judged as blocked.

`block_overtime` (`double`, default: 0)

- Time out of trigger block,It is used to prevent persisting in blocked mode.

`anti_block_angle` (`double`, default: 0)

- It means the anti angle of friction wheel.

`anti_block_threshold` (`double`, default: 0)

- It is used to judge if the ballistic blockage has been resolved, if the angle at which the trigger is reversed greater than this value, we can judge that the trigger blocked problem has been resolved.

`qd_10` (`double`, default: 0)

- It can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_10 on behalf of the speed of fire(10 m/s).

`qd_15` (`double`, default: 0.5)

+ It is the speed of friction wheel. The qd_15 on behalf of the speed of shooting(15 m/s)

`qd_18` (`double`, default: 0)

+ It is the speed of friction wheel. The qd_18 on behalf of the speed of shooting(18 m/s)

`qd_30` (`double`, default: 0)

+  It is the speed of friction wheel. The qd_30 on behalf of the speed of shooting(30 m/s)

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

#### 8.2. Complete description

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

## 9. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues) .