# rm_chassis_controllers

## 1. Overview

The Controller is RoboMaster robot chassis controller, balance, swerve and mecanum included.

***Keywords***: mecanum, swerve, balance, chassis.

### License
The source code is released under a [BSD 3-Clause license](http://192.168.0.100:7070/dynamicx/rm_gimbal_controllers/-/blob/master/LICENSE)
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
* [Robot Operating System (ROS) ](http://wiki.ros.org/) ( middleware for robotics )
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) ( linear algebra library )
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
* roscpp
* roslint
* rm_msgs 
* rm_common
* imu_sensor_controller
* robot_localization
* robot_state_controller
* rm_description
* rm_gazebo


#### 2.2.2. Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## 3. Usage

* Run the controller with mon launch:

      mon launch rm_chassis_controller load_controllers.launch
## 4. Config

* ***auto.yaml***: It loads some controllers and the parameters used for auto into the parameter server.
* ***balance.yaml***: It loads some controllers and the parameters used for balance into the parameter server.
* ***localization.yaml***: It loads some controllers and the parameters used for localization into the parameter server.
* ***engineer.yaml***: It loads some controllers and the parameters used for engineer robot into the parameter server.
* ***hero.yaml***: It loads some controllers and the parameters used for hero robot into the parameter server.
* ***sentry.yaml***: It loads some controllers and the parameters used for sentry robot into the parameter server.
* ***standard3.yaml***: It loads some controllers and the parameters used for standard3 robot into the parameter server.
* ***standard4.yaml***: It loads some controllers and the parameters used for standard4 robot into the parameter server.
* ***standard5.yaml***: It loads some controllers and the parameters used for standard5 robot into the parameter server.
## 5. Launch files

* load_controllers.launch: It loads tf, robot_localization and some controllers, robot_state_controller, joint_state_controller and chassis controller
## 6. ROS API

### 6.1. Description
The controller main input is a [geometry_msgs::Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) topic in the namespace of the controller.
### 6.2 Subscribed Topics

`base_imu`( [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) )
* Inertial Measurement Unit data.

`command`( )
* Velocity command.

`cmd_vel`( [geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) )
* Velocity command.

### 6.3 Published Topics

`state_real`( )

* Publish the real state.

`odom`( [nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) )
*  Odometry computed from the hardware feedback.
### 6.4 Parameters

#### 6.4.1 common

`joint_left_name` ( `string` | string [ ... ] )
* Left wheel joint name or list of joint names.

`joint_right_name` ( `string` | string [ ... ] ) 
* Right wheel joint name or list of joint names.  

`wheel_radius` ( `double`, default: 0.02 )
* Radius of the wheels. It is expected they all have the same size. The rm_chassis_controller will attempt to read the value from the URDF if this parameter is not specified.

`wheel_track` ( `double`, default: 0.410 )
* Distance between wheels. It is expected they all have the same size. The rm_chassis_controller will attempt to read the value from the URDF if this parameter is not specified.

`wheel_base` ( `double`, default: 0.320 )
* Distance between the axes. It is expected they all have the same size. The rm_chassis_controller will attempt to read the value from the URDF if this parameter is not specified.  

`twist_angular` ( `double`, default: M_PI / 6 )
* The velocity of angular.

`enable_odom_tf` ( `bool`, default: true )
* Publish to TF directly or not.

`twist_covariance_diagonal` ( double [ 6 ] )

* Diagonal of the covariance matrix for odometry twist publishing.

`publish_rate` ( `double`, default: 50 )
* Frequency ( in Hz ) at which the odometry is published. Used for both tf and odom.

`coeff` ( `double` )
* The power limit coeff.

`min_vel` ( `double` )
* Minimum angular velocity of single chassis wheel.

`timeout` ( `double` )
* Allowed period ( in s ) allowed between two commands.

#### 6.4.2 Balance

`joint_left_name` ( `string` | string [ ... ] )
* Left wheel joint name or list of joint names.

`joint_right_name` ( `string` | string [ ... ] )
* Right wheel joint name or list of joint names.  

`com_pitch_offset` ( `double`, default: 0 )
* The reduction ratio of pitch.

`a` ( `double [ 16 ]` )
* State space expression.

`b` ( `double [ 8 ]` )
* State space expression.

`q` ( `double [ 16 ]` )
* Weight matrix.

`r` ( `double [ 4 ]` )
* Weight matrix.

#### 6.4.3 Swerve
`modules`

* Data about each component.


## 7. Controller configuration examples

### 7.1. Minimal description
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
### 7.2. Complete description
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
## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues) .