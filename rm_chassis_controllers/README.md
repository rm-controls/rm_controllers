# rm_chassis_controllers

## Overview

There are three states: raw, follow and twist. The output torque and speed of each motor of the chassis can be
calculated according to the current state of the control, the received speed and pose of the pan/tilt, and the speed and
acceleration commands, and the data is returned by the motor to calculate The speed and posture of the chassis are
released. The control algorithm involved in the chassis controller is PID algorithm.

**Keywords:** mecanum, swerve, balance, chassis, ROS, RoboMaster

### Hardware interface type

+ `JointStateInterface` Used to get the position and speed of chassis wheel joint.

+ `EffortJointInterface` Used to send the torque command of chassis wheel joint.

+ `RoboSateInterface` Used for high-frequency maintenance of the transformation relationship of changing odom to
  base_link.

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

```
sudo apt-get install ros-noetic-rm-chassis-controllers
```

Or better, use `rosdep`:

```
sudo rosdep install --from-paths src
```

### Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org/) (middleware for robotics),
* roscpp
* rm_common
* controller_interface
* effort_controllers
* tf2_geometry_msgs
* angles
* robot_localization

## ROS API

#### Subscribed Topics

* **`base_imu`** ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))

  The inertial measurement unit data of base command.

* **`command`** (rm_msgs::ChassisCmd)

  Set the mode, acceleration, and maximum power of the chassis.

* **`cmd_vel`** ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

  Set the speed of the chassis.

#### Published Topics

* **`odom`**([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

  Chassis odometer information (speed, position, covariance).

* **`state`**([rm_msgs::BalanceState](http://docs.ros.org/en/api/rm_msgs/html/msg/BalanceState.html))

  Contains quantities of state and control about the Balance.

#### Parameters

##### common

* **`wheel_radius`** (double)

  Radius of the wheels.

* **`wheel_track`** (double)

  The distance between the center of the left and right wheels on the same side.

* **`wheel_base`** (double)

  The distance between the center of the front and rear wheels on the same side.

* **`twist_angle`** (double)

  Amplitude of twist at `twist` state.

* **`enable_odom_tf`** (bool, default: true)

  Option.If it is set to true, it will store Transform in tf_buffer.

* **`publish_odom_tf_`** (bool, default: false)

  Option.If it is set to true, enable_odom_tf is also true, it will send Transform from odom to base.

* **`twist_covariance_diagonal`** (double[6])

  The diagonal covariance matrix of twist.

* **`publish_rate`** (double, default: 50)

  Frequency (in Hz) of publishing Transform.

* **`coeff`** (double)

  A coefficient. Adjust this coefficient to reduce the impact of power loss.

* **`min_vel`** (double)

  The minimum velocity of chassis joint which is used to calculate the max torque.

* **`timeout`** (double)

  Allowed period (in s) between two commands. If the time is exceed this period, the speed of chassis will be set 0.

* **`power_offset`** (double)

  Fix the difference between theoretical power and actual power.

##### Balance

* **`imu_name`** (string, default: "base_imu")

  Chassis imu name.

* **`left/wheel_joint`** (string, default: "left_wheel_joint")

  left wheel joint name.

* **`left/block_joint`** (string, default: "left_momentum_block_joint")

  left momentum block joint name.

* **`right/wheel_joint`** (string, default: "right_wheel_joint")

  right wheel joint name.

* **`right/block_joint`** (string, default: "right_momentum_block_joint")

  right momentum block joint name.

* **`m_w`** (double, default: 0.72)

  mass of single wheel.

* **`m`** (double, default: 11.48)

  mass of the robot except wheels and momentum_blocks.

* **`m_b`** (double, default: 1.13)

  mass of single momentum_block.

* **`i_w`** (double, default: 0.01683)

  The moment of inertia of the wheel around the rotational axis of the motor.

* **`l`** (double, default: 0.0587)

  The vertical component of the distance between the wheel center and the center of mass of robot.

* **`y_b`** (double, default: 0.16)

  The y-axis component of the coordinates of the momentum block in the base_link coordinate system.

* **`z_b`** (double[4], default: 0.0468)

  The vertical component of the distance between the momentum block and the center of mass of robot.

* **`g`** (double, default: 9.8)

  Gravity constant.

* **`i_m`** (double, default: 0.1982)

  The moment of inertia of the robot around the y-axis of base_link coordinate.

* **`q`** (double[16])

  Weight matrix.

* **`r`** (double[4])

  Weight matrix.

##### Swerve

* **`/modules/<module_name>/position`** (double[2])

  The position of module.

* **`/modules/<module_name>/pivot/joint`** (string)

  Joint between chassis and privot.

* **`/modules/<module_name>/pivot/offset`** (double)

  Angle between the wheel rotation axis and the chassis's Y axis.

* **`/modules/<module_name>/wheel/joint`** (string)

  Joint between privot and wheel.

* **`/modules/<module_name>/wheel/radius`** (double)

  The radius of wheel.

##### Omni

* **`/wheels/<wheels_name>/pose`** (double[3])

  The pose of wheel.

* **`/wheels/<wheels_name>/joint`** (string)

  wheel joint name.

* **`/wheels/left_front/roller_angle`** (double)

  The roller angle of wheel.

* **`/wheels/left_front/radius`** (double)

  The radius of wheel.

## Controller configuration examples

### Complete description

```
  chassis_controller:
    type: rm_chassis_controllers/OmniController
    publish_rate: 100
    enable_odom_tf: true
    publish_odom_tf: false
    power:
      effort_coeff: 10.0
      vel_coeff: 0.003
      power_offset: -8.41
    twist_angular: 0.5233
    timeout: 0.1
    pid_follow: { p: 5.0, i: 0, d: 0.3, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
    twist_covariance_diagonal: [ 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ]

    wheels:
      left_front:
        pose: [ 0.147, 0.147, 2.356 ]
        joint: left_front_wheel_joint
        <<: &wheel_setting
          roller_angle: 0.
          radius: 0.07625
          pid: { p: 0.41, i: 0, d: 0.0, i_max: 0.0, i_min: 0.0, antiwindup: true, publish_state: true }
      right_front:
        pose: [ 0.147, -0.147, 0.785 ]
        joint: right_front_wheel_joint
        <<: *wheel_setting
      left_back:
        pose: [ -0.147, 0.147, -2.356 ]
        joint: left_back_wheel_joint
        <<: *wheel_setting
      right_back:
        pose: [ -0.147, -0.147, -0.785 ]
        joint: right_back_wheel_joint
        <<: *wheel_setting
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
