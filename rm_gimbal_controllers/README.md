# rm_gimbal_controllers

## Overview

The rm_gimbal_controllers has three states: RATE, TRACK, and DIRECT. It performs PID control on the yaw joint and pitch joint according to commands. It can also perform moving average filtering based on detection data and calculate, predict and track targets based on the ballistic model.

**Keywords:** ROS, Robomaster, gimbal, bullet solver, moving average filter

### License

The source code is released under a [ BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Hardware interface type
+ `JointStateInterface` Used to get the speed and position of gimbal joint.
+ `EffortJointInterface` Used to send effort command to gimbal joint .
+ `RoboStateInterface` Used to get the current and historical transform between gimbal and the world coordinate system and the transform between visual target and the world coordinate system.

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-rm-gimbal-controllers

Or better, use `rosdep`:

    sudo rosdep install --from-paths src

### Dependencies
* roscpp
* rm_common
* effort_controllers
* tf2_eigen
* tf2_geometry_msgs
* visualization_msgs
* dynamic_reconfigure

## ROS API

#### Subscribed Topics
* **`command`** (rm_msgs/GimbalCmd)

  Set gimbal mode, pitch and yaw axis rotation speed, tracking target, pointing target and coordinate system.

* **`/detection`** (rm_msgs/TargetDetectionArray)

  Receive visual recognition data.

* **`/<camera_name>/camera_info`** (CameraInfo)

  Make sure that the detection node receives a new frame of image and sends the prediction data to the detection node.

#### Published Topics
* **`error`** (rm_msgs/GimbalDesError)
  The error calculated by the ballistic model to shoot at the current gimbal angle to the target.

* **`track`** (rm_msgs/TrackDataArray)
  The predicted data used for detection node to decide the ROI.
##### Bullet solver
* **`model_desire`** ( [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) )
  Used to visualize the desired bullet trajectory.

* **`model_real`** ( [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) )
  Used to visualize the trajectory that caculated by ballistic model in the current gimbal angle.
#### Parameters
* **`detection_topic`** (string, default: "/detection")

  The name of the topic where detection node gets predicted data.

* **`detection_frame`** (string, default: "detection")

  The name of the frame of detection.

* **`camera_topic`** (string, default: "/galaxy_camera/camera_info")

  The name of the topic that is determined that the detection node receives a new frame of image and sends the prediction data to the detection node.

* **`publish_rate`** (double)

  Frequency (in Hz) of publishing gimbal error.

* **`chassis_angular_data_num`** (double)

  The number of angle data of chassis.Used for chassis angular average filter.

* **`time_compensation`** (double)

  Time(in s) of image transmission delay(in s).Used to compensate for the effects of images transimission delay

##### Bullet solver
_Bullet solver is used to get the bullet drop point_
* **`resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18, resistance_coff_qd_30`** ( `double` )

  The air resistance coefficient used for bullet solver when bullet speed is 10 m/s, 15 m/s, 16 m/s, 18 m/s and 30 m/s.

* **`g`** (double)

  Acceleration of gravity.

* **`delay`** (double)

  Shooting delay time(in s) after shooter get the shooting command.Used to compensate for the effects of launch delay.

* **`timeout`** (double)

  Timeout time((in s)) of bullet solver.Used to judge whether bullet solver can caculate the bullet drop point.

##### Moving average filter
_Moving average filter is used for filter the target armor center when target is spin._
* **`is_debug`** ( `bool`, default: true )

  The debug status.When it is true, debug data will be pulished on the filter topic.

* **`pos_data_num`** (double, default: 20)

  The number of armor position data.

* **`vel_data_num`** (double, default: 30)

  The number of armor linear velocity data.

* **`gyro_data_num`** (double, default: 100)

  The number of target rotation speed data.

* **`center_data_num`** (double, default: 50)

  The number of target rotation center position data.

* **`center_offset_z`** (double)

  Offset(in meter) on the z axis.Used to compensate for the error of filter result on z axis.

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
