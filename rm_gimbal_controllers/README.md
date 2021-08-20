# rm_gimbal_controllers

## 1. Overview

The Controller is RoboMaster robot gimbal controller. It is used for ballistic solution and gimbal moving.

***Keywords***: gimbal, ballistic solution, ROS

### License
The source code is released under a [ BSD 3-Clause license](http://192.168.0.100:7070/dynamicx/rm_gimbal_controllers/-/blob/master/LICENSE).
#### Author: DynamicX
#### Affiliation: DynamicX

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
### 1.1. Hardware interface type
The controller works with pitch joints and yaw joints through a **velocity** interface.

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
* visualization_msgs
* dynamic_reconfigure


#### 2.2.2. Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## 3. Usage

* Run the controller with mon launch:

      mon launch rm_gimbal_controller load_controllers.launch

## 4. Cfg
* ***BulletSolver.cfg***: It is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface.
* ***Gimbal.cfg***: It is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface.

## 5. Config

* ***engineer.yaml***: It loads some controllers and the parameters used for engineer robot into the parameter server.
* ***hero.yaml***: It loads some controllers and the parameters used for hero robot into the parameter server.
* ***sentry.yaml***: It loads some controllers and the parameters used for sentry robot into the parameter server.
* ***standard3.yaml***: It loads some controllers and the parameters used for standard3 robot into the parameter server.
* ***standard4.yaml***: It loads some controllers and the parameters used for standard4 robot into the parameter server.
* ***standard5.yaml***: It loads some controllers and the parameters used for standard5 robot into the parameter server.

## 6. Launch files

* ***load_controller.launch***: It loads tf and some controllers, robot_state_controller, joint_state_controller, lower_gimbal_controller, upper_gimbal_controller is included.

## 7. ROS API

### 7.1. Description
The controller main input is a [geometry_msgs::Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) topic in the namespace of the controller.
### 7.2. Subscribed Topics
* `command` ()

  Velocity command.

* `detection_topic` ()

  Object command.

* `camera_topic` ()

  Image or video command.

### 7.3. Published Topics
* `model_desire` ( [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) )
  Display target route of the model.

* `model_real` ( [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) )

  Display real route of the model.

* `id`
  Publish the id of target.

* `error_des`
  Process error information.

* `track`
  Process the track array.
### 7.4. Parameters
* `resistance_coff_qd_10` ( `double`, default: 0 )

  The air resistance coeff when bullet speed is 10 m/s.

* `resistance_coff_qd_15` ( `double`, default: 0 )

  The air resistance coeff when bullet speed is 15 m/s.

* `resistance_coff_qd_16` ( `double`, default: 0 )

  The air resistance coeff when bullet speed is 16 m/s.

* `resistance_coff_qd_18` ( `double`, default: 0 )

  The air resistance coeff when bullet speed is 18 m/s.

* `resistance_coff_qd_30` ( `double`, default: 0 )

  The air resistance coeff when bullet speed is 30 m/s.

* `g` ( `double`, default: 0 )

  The value of acceleration of gravity.

* `dt` ( `double`, default: 0 )

  The duration of sending data.

* `delay` ( `double`, default: 0 )

  Bullet launch delay.

* `timeout` ( `double`, default: 0 )

  Timeout time of bullet model solution.

* `detection_topic` ( `string` | string [ ... ] )

  The name of a topic about some form of detection.

* `camera_topic` ( `string` | string [ ... ] )

  The name of a topic about some form of camera info.

* `detection_frame` ( `string` | string [ ... ] )

  The name of the frame of detection.

* `publish_rate` ( `double`, default: 50 )

  Frequency ( in Hz ) at which the odometry is published. Used for both tf and odom.

* `chassis_angular_data_num` ( `double` )

  Deflection angle of chassis.

* `time_compensation` ( `double`, default: 0 )

  Time of image transmission delay.

* `is_debug` ( `bool`, default: true )

  The debug status.

* `pos_data_num` ( `double` )

  The data of filter position.

* `vel_data_num` ( `double` )

  The data of filter velocity.

* `center_data_num` ( `double` )

  The data of filter center.

* `gyro_data_num` ( `double` )

  The data of filter gyro velocity.

* `center_offset_z` ( `double` )

  The filter center reduction ratio.

## 8. Controller configuration examples

### 8.1. Minimal description
```
gimbal_controller:
    type: rm_gimbal_controllers/Controller
    time_compensation: 0.03
    publish_rate: 100
    chassis_angular_data_num: 20
    camera_topic: "/galaxy_camera/camera_info"
    yaw:
      joint: "yaw_joint"
      pid: { p: 8, i: 0, d: 0.4, i_clamp_max: 0.0, i_clamp_min: -0.0, antiwindup: true, publish_state: true }
    pitch:
      joint: "pitch_joint"
      pid: { p: 10, i: 50, d: 0.3, i_clamp_max: 0.4, i_clamp_min: -0.4, antiwindup: true, publish_state: true }
```
### 8.2. Complete description
```
gimbal_controller:
    type: rm_gimbal_controllers/Controller
    time_compensation: 0.03
    publish_rate: 100
    chassis_angular_data_num: 20
    camera_topic: "/galaxy_camera/camera_info"
    yaw:
      joint: "yaw_joint"
      pid: { p: 8, i: 0, d: 0.4, i_clamp_max: 0.0, i_clamp_min: -0.0, antiwindup: true, publish_state: true }
    pitch:
      joint: "pitch_joint"
      pid: { p: 10, i: 50, d: 0.3, i_clamp_max: 0.4, i_clamp_min: -0.4, antiwindup: true, publish_state: true }
    bullet_solver:
      resistance_coff_qd_10: 0.45
      resistance_coff_qd_15: 0.1
      resistance_coff_qd_16: 0.7
      resistance_coff_qd_18: 0.55
      resistance_coff_qd_30: 3.0
      g: 9.81
      delay: 0.1
      dt: 0.001
      timeout: 0.001
      publish_rate: 50
    moving_average_filter:
      is_debug: true
      center_offset_z: 0.05
      pos_data_num: 20
      vel_data_num: 30
      center_data_num: 50
      gyro_data_num: 100
```


## 9. Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues).
