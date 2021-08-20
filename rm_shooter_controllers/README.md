# rm_shooter_controller

## 1.Overview

The Controller is RoboMaster robot shooter controller. It is used for reading joint sensor data and sending command to motors. Besides , It will calls control loop (update method) periodically at a set frequency .

**Keywords:** shooter

#### License

The source code is released under a [BSD 3-Clause license]().

**Author: DynamicX **

**Affiliation: DynamicX **

**Maintainer: DynamicX **

The rm_shooter_controller package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### 1.1. Hardware interface type

The controller works with friction wheel joints through a **velocity** interface.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

## 2.Installation

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

+ **shooter.cfg:** The following shooter.cfg is used for adding parameters to rqt plugin that you can dynamically adjust parameters in rqt ui interface

## 5. Config

Config file config

+ **hero.yaml:** it loads some controllers and the parameters used for hero robot into the parameter server

+ **sentry.yaml:** it loads some controllers and the parameters used for sentry robot into the parameter server

+ **standard3.yaml:** it loads some controllers and the parameters used for standard3 robot into the parameter server

+ **standard4.yaml:** it loads some controllers and the parameters used for standard4 robot into the parameter server

+ **standard5.yaml:** it loads some controllers and the parameters used for standard5 robot into the parameter server

## 6. Launch files

- **load_controller.launch:** you can launch robot_state_controller controllers/joint_state_controller controllers/upper_gimbal_controller
  controllers/lower_gimbal_controller controllers/upper_shooter_controller controllers/lower_shooter_controller and load the paramters into parameter server by launch files.

## 7. ROS API

### 7.1. Description

The controller main input is a [geometry_msgs::Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) topic in the namespace of the controller.

#### 7.2. Subscribed Topics

`command` ()

- Velocity command.

#### 7.3. Published Topics

`odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

- Odometry computed from the hardware feedback.

`/tf` ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))

- Transform from odom to base_footprint

`publish_cmd` ([geometry_msgs/TwistStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html))

- Available when "publish_cmd" parameter is set to True. It is the Twist after limiters have been applied on the controller input.

#### 7.4. Parameters

`block_effort` (`double`, default: 0)

+ Upper limit moment of blocked cartridge

`block_speed` (`double`, default: 0)

- Upper limit speed of blocked ammunition

`block_duration` (`double`, default: 0)

- The Jam duration of blocked ammunition

`block_overtime` (`double`, default: 0)

- Time out of blocked ammunition

`anti_block_angle` (`double`, default: 0)

- The anti angle of friction wheel

`anti_block_threshold` (`double`, default: 0)

- It used to judge if the ballistic blockage has been resolved

`qd_10` (`double`, default: 0)

- It means q dot which can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_10 on behalf of the rate of fire(10 m/s)

`qd_15` (`double`, default: 0.5)

- It means q dot which can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_15 on behalf of the rate of fire(15 m/s)

`qd_18` (`double`, default: 0)

- It means q dot which can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_18 on behalf of the rate of fire(18 m/s)

`qd_30` (`double`, default: 0)

- It means q dot which can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_30 on behalf of the rate of fire(30 m/s)

`robot_state_controller` (`type: robot_state_controller/RobotStateController`)

- Receive joint_ state_ controller publishes topic messages and publishes theTF results.

`joint_state_controller` (`type: joint_state_controller/JointStateController`)

- This controller is which ros_control provides us with a default controller for publishing / joint_ States topic data.

`shooter_controller` (`type: rm_shooter_controllers/Controller`)

- It is used to configure relevant parameters for the shooter controller.

- publish_rate: 50

- friction_left(`joint: left_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

- friction_right(`joint: right_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

- trigger(`joint: trigger_joint`)

  ```
  pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
  ```

`upper_shooter_controller` (`type: rm_shooter_controllers/Controller`)

+ This controller is only used for controlling the sentry robot's upper barrel and configure relevant parameters for the upper shooter controller

+ publish_rate: 50

+ friction_left(`joint: upper_left_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

+ friction_right(`joint: upper_right_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

+ trigger(`joint: upper_trigger_joint`)

  ```
  pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
  ```

`lower_shooter_controller` (`type: rm_shooter_controllers/Controller`)

+ This controller is only used for controlling the sentry robot's lower barrel and configure relevant parameters for the lower shooter controller

+ publish_rate: 50

+ friction_left(`joint: lower_left_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

+ friction_right(`joint: lower_right_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

+ trigger(`joint: lower_trigger_joint`)

  ```
  pid: { p: 50.0, i: 0.0, d: 1.5, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
  ```

### 8. Controller configuration examples

#### 8.1. Minimal description

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
