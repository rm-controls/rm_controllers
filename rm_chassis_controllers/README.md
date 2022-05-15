# rm_chassis_controllers

## Overview

There are four states: raw, follow, gyro and twist. The output torque and speed of each motor of the chassis can be calculated according to the current state of the control, the received speed and pose of the pan/tilt, and the speed and acceleration commands, and the data is returned by the motor to calculate The speed and posture of the chassis are released. The control algorithm involved in the chassis controller is PID algorithm.

**Keywords:** mecanum, swerve, reaction wheel, chassis, sentry, omni, ROS, RoboMaster

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

### Building from Source

#### Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org/) (middleware for robotics), 
* roscpp
* roslint 
* rm_msgs 
* rm_common 
* nav_msgs 
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

#### Building

* Build this package with catkin build. Clone the latest version from this repository into your catkin workspace.

```
catkin_workspace/src
git clone https://github.com/rm-controls/rm_controllers.git
rosdep install --from-paths . --ignore-src
catkin build
```

## Usage

Run the controller with mon launch:

```
mon launch rm_chassis_controllers load_controllers.launch
```

## Launch files

* **load_controllers.launch:** It loads tf, robot_localization and some controllers, robot_state_controller, joint_state_controller and chassis controller are included.

## ROS API

#### Subscribed Topics

* **`command`** (rm_msgs::ChassisCmd)

  Set the mode, acceleration, and maximum power of the chassis.

* **`cmd_vel`** ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

  Set the speed of the chassis.

#### Published Topics
* **`odom`**([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

  Chassis odometer information (speed, position, covariance).

#### Parameters

##### chassis_base

* **`wheel_radius`** (double)

  Radius of the wheels.

* **`wheel_track`** (double)

  The distance between the center of the left and right wheels on the same side.

* **`wheel_base`** (double)

  The distance between the center of the front and rear wheels on the same side.

* **`twist_angle`** (double)

  Amplitude of twist at `twist` state.

* **`enable_odom_tf`** (bool, default: true)

  Option.If set this param true, it will enable send Transform from odom to base.

* **`publish_rate`** (double, default: 50)

  Frequency (in Hz) of publishing Transform.

* **`timeout`** (double)

  Allowed period (in s) between two commands. If the time is exceed this period, the speed of chassis will be set 0.

* **`effort_coeff_`** (double)

  A coefficient used to compute "zoom_coeff" which is used to adjust the chassis power.

* **`vel_coeff_`**(double)

  A coefficient used to compute "zoom_coeff" which is used to adjust the chassis power.

* **`power_offset_`**(double)

  The offset of chassis power.

* **`publish_odom_tf_`** (bool, default: false)

  Option.If set this param true, it will send Transform from odom to base.

* **`twist_covariance_diagonal`**(double[6])

  The diagonal covariance matrix of twist.
##### Reaction_wheel

* **`imu_name`** (string)

  Imu name.

* **`joint`** (string)

  Joint names.

* **`q`** (double[16])

  Weight matrix.

* **`r`** (double[4])

  Weight matrix.

* **`alpha_`**(double)

  Used to compute pitch_offset.

* **`m_b`** (double)

  Mass of the pendulum body.

* **`m_w`** (double)

  Mass of the wheel.

* **`l_b`** (double)

  The distance between the center of mass of the pendulum body and the pivot point.

* **`l`** (double)

  The distance between the motor axis and the pivot point.

* **`i_b`** (double)

  The moment of inertia of the pendulum body around the pivot point.

* **`i_w`** (double)

  The moment of inertia of the wheel around the rotational axis of the motor

* **`g`** (double)

  Negative value of gravitational acceleration.

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

##### Sentry
* **`catapult_angle_`**(double)

  Angle of catapult. Whether the angle is positive or negative depends on the direction of the chassis speed.

* **`lock_duratoin_`**(double)

  Lock duration of the catapult.

* **`velocity_coefficient`**(double)

  A coefficient used to judge whether enter normal mode or not.

##### Omni
* **`chassis_radius`**(double)

  Radius of the omni wheel chassis，it is used to compute the chassis angular velocity.

## Controller configuration examples

### Complete description

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

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
