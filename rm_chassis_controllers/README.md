# rm_chassis_controllers

## 1. Overview
The Controller is RoboMaster robot chassis controller, balance, swerve and mecanum are included. It controls the speed, power and posture of the chassis.

**Keywords:** mecanum, swerve, balance, chassis.

#### License
The source code is released under a [BSD 3-Clause license]().
 
**Author: DynamicX**

**Affiliation: DynamicX**

**Maintainer: DynamicX**

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### 1.1. Hardware interface type

The controller works with friction wheel joints through a **effort** interface.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

## 2. Installation

### 2.1. Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-rm-chassis-controllers

Or better, use `rosdep`:

    sudo rosdep install --from-paths src

### 2.2. Building from Source

#### 2.2.1. Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org/) (middleware for robotics),
* roscpp
* roslint
* rm_msgs
* rm_common
* pluginlib
* hardware_interface
* controller_interface
* forward_command_controller
* realtime_tools
* control_toolbox
* effort_controllers
* tf2
* tf2_geometry_msgs
* angles
* imu_sensor_controller
* robot_localization


#### 2.2.2. Building

* Build this package with catkin build. Clone the latest version from this repository into your catkin workspace.
```
catkin_workspace/src
git clone https://github.com/rm-controls/rm_controllers.git
rosdep install --from-paths . --ignore-src
catkin build
```


## 3. Usage

Run the controller with mon launch:

      mon launch rm_chassis_controllers load_controllers.launch

## 4. Launch files

* **load_controllers.launch:** It loads tf, robot_localization and some controllers, robot_state_controller,
  joint_state_controller and chassis controller are included.
  
## 5. ROS API

### 5.1 Subscribed Topics

* `base_imu`([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))

  The inertial measurement unit data of base command.
   
* `command`(rm_msgs::ChassisCmd)

  Velocity command.

* `cmd_vel`([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

  Velocity command.

### 5.2 Published Topics

* `state_real`(rm_msgs::BalanceState）

  Publish the real state.

* `odom`([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

  Odometry computed from the hardware feedback.

### 5.3 Parameters

#### 5.3.1 common

* `wheel_radius`(double, default: 0.02)

  Radius of the wheels.

* `wheel_track`(double, default: 0.410)

  The distance between the center of the front (rear) two wheels.

* `wheel_base`(double, default: 0.320)

  The distance between the center of axle.

* `twist_angular`(double, default: M_PI / 6)

  Angle of distortion.

* `enable_odom_tf`(bool, default: true)

  Publish to TF directly or not.

* `twist_covariance_diagonal`(double[6])

  The diagonal covariance matrix of twist.

* `publish_rate`(double, default: 50)

  Frequency (in Hz) at which the topic is published.

* `coeff`(double)

  Power factor.

* `min_vel`(double)

  Minimum velocity at the power.

* `timeout`(double)

  Allowed period (in s) between two commands. If the time is exceed this period, it will turn off.

#### 5.3.2 Balance

* `joint_left_name`(string, default: "joint_left")

  Left wheel joint name or list of joint names.

* `joint_right_name`(string, default: "joint_right")

  Right wheel joint name or list of joint names.


* `com_pitch_offset`(double, default: 0)

  The reduction ratio of pitch.

* `a`(double[16])

  State space expression.

* `b`(double[8])

  State space expression.

* `q`(double[16])

  Weight matrix.

* `r`(double[4])

  Weight matrix.

#### 5.3.3 Swerve

* `/modules/left_front/position`(double[2])

  The position of left front wheel.
* `/modules/left_front/pivot/offset`(double)

  The reduction ratio of left front pivot.
* `/modules/left_front/wheel/radius`(double)

  The radius of left front wheel.
* `/modules/right_front/position`(double[2])

  The position of right front wheel.
* `/modules/left_back/position`(double[2])

  The position of left back wheel.
* `/modules/right_back/position`(double[2])

  The position of right back wheel.

## 6. Controller configuration examples

### 6.1. Minimal description

```
chassis_controller:
type: rm_chassis_controllers/MecanumController
publish_rate: 100
enable_odom_tf: true
wheel_radius: 0.07625
left_front:
joint: "left_front_wheel_joint"
pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
right_front:
joint: "right_front_wheel_joint"
pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
left_back:
joint: "left_back_wheel_joint"
pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
right_back:
joint: "right_back_wheel_joint"
pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
twist_covariance_diagonal: [ 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ]
```
### 6.2. Complete description
```
chassis_controller:
    type: rm_chassis_controllers/MecanumController
    publish_rate: 100
    enable_odom_tf: true
    wheel_radius: 0.07625
    left_front:
      joint: "left_front_wheel_joint"
      pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
    right_front:
      joint: "right_front_wheel_joint"
      pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
    left_back:
      joint: "left_back_wheel_joint"
      pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
    right_back:
      joint: "right_back_wheel_joint"
      pid: { p: 0.8, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
    twist_covariance_diagonal: [ 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ]

    wheel_base: 0.395
    wheel_track: 0.374
    power:
      coeff: 0.535
      min_vel: 4.4
    twist_angular: 0.5233
    timeout: 0.1
    pid_follow: { p: 5, i: 0, d: 0.8, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
```
## 7. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues) .
