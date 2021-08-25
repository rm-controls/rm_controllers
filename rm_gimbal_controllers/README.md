# rm_gimbal_controllers

## Overview

The Controller is RoboMaster robot gimbal controller. It is used for ballistic solution and gimbal moving.

***Keywords***: gimbal, ballistic solution, ROS

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
* **BulletSolver.cfg**: It is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface.
* **Gimbal.cfg**: It is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface.

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

  The name of a topic about some form of detection.

* **`camera_topic`** ( `string` | string [ ... ] )

  The name of a topic about some form of camera info.

* **`detection_frame`** ( `string` | string [ ... ] )

  The name of the frame of detection.

* **`publish_rate`** ( `double`, default: 50 )

  Frequency ( in Hz ) at which the odometry is published. Used for both tf and odom.

* **`chassis_angular_data_num`** ( `double` )

  Deflection angle of chassis.

* **`time_compensation`** ( `double`, default: 0 )

  Time of image transmission delay.

##### Bullet solver
_Bullet solver is used to get the bullet point_
* **`resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18, resistance_coff_qd_30`** ( `double` )

  The air resistance coeff when bullet speed is 10 m/s, 15 m/s, 16 m/s, 18 m/s and 30 m/s.Used for ballistic model to calculate the bullet point.

* **`g`** ( `double`, default: 0 )

  The value of acceleration of gravity.

* **`dt`** ( `double`, default: 0 )

  The duration of sending data.

* **`delay`** ( `double`, default: 0 )

  Bullet launch delay.

* **`timeout`** ( `double`, default: 0 )

  Timeout time of bullet model solution.

##### Moving average filter
* **`is_debug`** ( `bool`, default: true )

  The debug status.

* **`pos_data_num`** ( `double` )

  The data of filter position.

* **`vel_data_num`** ( `double` )

  The data of filter velocity.

* **`center_data_num`** ( `double` )

  The data of filter center.

* **`gyro_data_num`** ( `double` )

  The data of filter gyro velocity.

* **`center_offset_z`** ( `double` )

  The filter center reduction ratio.

## Controller configuration examples

### Complete description
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
