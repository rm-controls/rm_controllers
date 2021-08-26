# rm_gimbal_controllers

## Overview

The rm_gimbal_controllers has three states: RATE, TRACK, and DIRECT. It performs PID control on the two joints of yaw and pitch according to commands. It can also perform moving average filtering based on visual target data and calculate, predict and track targets based on the bullet launch model.

**Keywords**: gimbal, ballistic solution, ROS, moving average filter

### License

The source code is released under a [ BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Hardware interface type
+ `JointStateInterface` Used to obtain the speed and position of gimbal joint.
+ `EffortJointInterface` Used to send effort command to gimbal joint .
+ `RoboStateInterface` Used to obtain the current and historical transform between gimbal and the world coordinate system and the transform between visual target and the world coordinate system.

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-gimbal-controllers

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source
#### Dependencies
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
* visualization_msgs
* dynamic_reconfigure

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## Usage

* Run the controller with mon launch:

      mon launch rm_gimbal_controller load_controllers.launch

## Cfg
* **BulletSolver.cfg**: Add parameters related to ballisitc model to each bullet speed.
* **Gimbal.cfg**: Add parameters related to image transimission delay.

## Launch files

* **load_controllers.launch**: Load the parameters in config files and load tf and robot_state_controller, joint_state_controller, gimbal_controller.

## ROS API

#### Subscribed Topics
* **`command`** (rm_msgs/ChassisCmd)

  Set gimbal mode, pitch and yaw axis rotation speed, tracking target, pointing target and coordinate system.

* **`/detection`** (rm_msgs/TargetDetectionArray)

  Receive visual recognition data.

* **`/<camera_name>/camera_info`** (CameraInfo)

  Make sure that the detection node receives a new frame of image and sends the prediction data to the detection node.

#### Published Topics
* **`error`** (rm_msgs/GimbalError)
  The distance error calculated by the ballistic model to shoot at the current gimbal angle to the target.

* **`track`** (rm_msgs/TrackDataArray)
  The predicted data used for detection node to decide the ROI.

#### Parameters
* **`detection_topic`** ( `string` | string [ ... ] )

  The name of the topic where detection node gets predicted data.

* **`camera_topic`** ( `string` | string [ ... ] )

  The name of the topic that return whether camera get a new frame.

* **`detection_frame`** ( `string` | string [ ... ] )

  The name of the frame of detection.

* **`publish_rate`** ( `double`, default: 50 )

  Frequency ( in Hz ) at which the odometry is published. Used for both tf and odom.

* **`chassis_angular_data_num`** ( `double` )

  The number of angle data of chassis.Used for chassis angular average filter.

* **`time_compensation`** ( `double`, default: 0 )

  Time(in s) of image transmission delay(in s).Used to compensate for the effects of images transimission delay

##### Bullet solver
_Bullet solver is used to get the bullet point_
* **`resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18, resistance_coff_qd_30`** ( `double` )

  The air resistance coeff used for bullet solver when bullet speed is 10 m/s, 15 m/s, 16 m/s, 18 m/s and 30 m/s.

* **`g`** ( `double`, default: 0 )

  Its value equal air resistance divided by mass.

* **`delay`** ( `double`, default: 0 )

  Bullet launch delay time(in s) after shooter get the shooting command.Used to compensate for the effects of launch delay.

* **`timeout`** ( `double`, default: 0 )

  Timeout time((in s)) of ballistic model solution.Used to judge whether bullet solver can caculate the bullet point.

##### Moving average filter
_Moving average filter is used for filter the target armor center when target is spin._
* **`is_debug`** ( `bool`, default: true )

  The debug status.When it is true, debug data will be pulished on the filter topic.

* **`pos_data_num`** ( `double` )

  The number of velocity data.

* **`vel_data_num`** ( `double` )

  The number of velocity data.

* **`center_data_num`** ( `double` )

  The number of armor center position data.

* **`gyro_data_num`** ( `double` )

  The target rotation speed data.

* **`center_offset_z`** ( `double` )

  Offset on the z axis.Used to compensate for the error of filter result on z axis.

## Controller configuration examples

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


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
