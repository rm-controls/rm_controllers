# rm_shooter_controller

## 1.Overview

The Controller is RoboMaster robot shooter controller. It is used for reading joint sensor data and sending command to motors. Besides , It will calls control loop (update method) periodically at a set frequency .

**Keywords:** shooter

#### License

The source code is released under a [BSD 3-Clause license]().

**Author: DynamicX**

**Affiliation: DynamicX**

**Maintainer: DynamicX**

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

+ **shooter.cfg:** The following shooter.cfg is used for adding parameters to rqt plugin that allows users to dynamically adjust parameters through the rqt ui interface when the node is running, and observe the impact of specific parameters on the algorithm in the node in real time. The parameters including block_effort, block_duration, block_speed, block_overtime, anti_block_angle, anti_block_threshold, qd_10, qd_15, qd_16, qd_18, qd_30.

## 5. Config

Config file config

+ **hero.yaml:** The hero.yaml loads the robot_state_controller,joint_state_controller and shooter_controller,besides it also loads the parameters used for hero robot into the parameter server which including the following, the publishing rate of each controller, push_per_rotation,push_qd_threshold,block_effort,block_speed,block_duration,block_overtime,anti_block_angle,anti_block_threshold,qd_16,lf_extra_rotat_speed. The three components of pid also set for left_friction_wheel_joint and right_friction_wheel_joint and trigger_joint .

+ **sentry.yaml:** The sentry.yaml loads four controllers involving robot_state_controller,joint_state_controller,upper_shooter_controller,lower_shooter_controller for sentry robot. The parameters loaded by this file involving the publishing rate of each controller, the components of pid that set for upper_left_friction_wheel_joint and upper_right_friction_wheel_joint and upper_trigger_joint, as same as the upper_shooter_controller,It also set the components of pid for the three joints of lower_shooer_zcontroller: lower_left_friction_wheel_joint and lower_right_friction_wheel_joint and lower_trigger_joint. In addition,the parameters loaded by this file also including push_per_rotation, push_qd_threshold, block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold, qd_16, lf_extra_rotat_speed

+ **standard3.yaml:** The standard3.yaml loads four controllers which including robot_state_controller, joint_state_controller, shooter_controller for No. 3 standard. The parameters that No. 3 standard needed to be loaded involving the publishing rate of each controller, the components of pid, and push_per_rotation, push_qd_threshold, block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold, qd_16, lf_extra_rotat_speed

+ **standard4.yaml:** The standard4.yaml loads four controllers which including robot_state_controller, joint_state_controller, shooter_controller for No. 4 standard. The parameters that No. 3 standard needed to be loaded involving the publishing rate of each controllers, the components of pid set for shooter_controller's three joints: left_friction_wheel_joint and right_friction_wheel_joint and trigger_joint, and push_per_rotation, push_qd_threshold, block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold, qd_16, lf_extra_rotat_speed

+ **standard5.yaml:** The standard5.yaml loads four controllers which including robot_state_controller, joint_state_controller, shooter_controller for No. 5 standard. The parameters that No. 3 standard needed to be loaded involving the publishing rate of each controllers, the components of pid set for shooter_controller's three joints: left_friction_wheel_joint and right_friction_wheel_joint and trigger_joint, and push_per_rotation, push_qd_threshold, block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold, qd_16, lf_extra_rotat_speed

## 6. Launch files

- **load_controller.launch:** you can launch robot_state_controller controllers/joint_state_controller controllers/upper_gimbal_controller
  controllers/lower_gimbal_controller controllers/upper_shooter_controller controllers/lower_shooter_controller and load the paramters into parameter server by launch files.

## 7. ROS API

### 7.1. Description

The controller main input is a [geometry_msgs::Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) topic in the namespace of the controller.

#### 7.2. Subscribed Topics

* `command` ()

  Velocity command.

#### 7.3. Parameters

* `block_effort` (`double`, default: 0)

  Upper limit moment of trigger block effort, Its minimum value is 0.0 and its maximum value is  10

* `block_speed` (`double`, default: 0)

  Lowest limit speed of  speed, If the speed is lower than this speed, it would be judged as blocked.

* `block_duration` (`double`, default: 0)

  The jam duration of blocked ammunition.If the jam time is over this duration, it would be judged as blocked.

* `block_overtime` (`double`, default: 0)

  Time out of trigger block.It is used to prevent persisting in blocked mode.

* `anti_block_angle` (`double`, default: 0)

  The anti angle of friction wheel.

* `anti_block_threshold` (`double`, default: 0)

  It is used to judge if the ballistic blockage has been resolved,if the angle at which the trigger is reversed greater than this value,we can judge that the trigger block problem has been resolved.

* `qd_10` (`double`, default: 0)

  It can be Interpreted as Joint angular velocity. It is the speed of friction wheel. The qd_10 on behalf of the rate of fire(10 m/s).

* `qd_15` (`double`, default: 0.5)

  It is the speed of friction wheel. The qd_15 on behalf of the rate of shooting(15 m/s).

* `qd_18` (`double`, default: 0)

  It is the speed of friction wheel. The qd_18 on behalf of the rate of shooting(18 m/s).

* `qd_30` (`double`, default: 0)

  It is the speed of friction wheel. The qd_30 on behalf of the rate of shooting(30 m/s).

* `robot_state_controller` (`type: robot_state_controller/RobotStateController`)

  It is used to receive joint_ state_ controller publishes topic messages and publishes the TF results.

* `joint_state_controller` (`type: joint_state_controller/JointStateController`)

  This controller is which ros_control provides us with a default controller for publishing / joint_ States topic data.

* `shooter_controller` (`type: rm_shooter_controllers/Controller`)

  It is used to configure relevant parameters for the shooter controller.

* `publish_rate` (`int`, default: 50)

  Command pulish rate.

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

* `upper_shooter_controller` (`type: rm_shooter_controllers/Controller`)

  This controller is only used for controlling the sentry robot's upper barrel and configure relevant parameters for the upper shooter controller



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

* `lower_shooter_controller` (`type: rm_shooter_controllers/Controller`)

  This controller is only used for controlling the sentry robot's lower barrel and configure relevant parameters for the lower shooter controller.

* friction_left(`joint: lower_left_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

* friction_right(`joint: lower_right_friction_wheel_joint`)

  ```
  pid: { p: 0.001, i: 0.01, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  ```

* trigger(`joint: lower_trigger_joint`)

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
