# robot_state_controller

## Overview

The robot_state_controller uses the URDF specified by the parameter robot_description and the joint positions from the `hardware_interface::JointStateInterface` to calculate the forward kinematics of the robot and publish the results via tf, and reads and manages the external incoming tf.

**keywords:** ROS, urdf, transformation, joint

#### License

The source code is released under a [BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The robot_state_controller package has been tested under [ROS](http://www.ros.org) Melodic and Noetic on respectively 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Hardware interface type

+ `JointStateInterface` Used to get the positions of different joints.
+ `RobotStateInterface` Used to obtain and maintain the transformation relationship of each link of the entire robot.

## Installation

### Installation from Packages

To install all packages from this repository as Debian packages use

```shell
sudo apt-get install ros-noetic-robot-state-controller
```

or better use `rosdep`:

```shell
sudo rosdep install --from-paths src
```

### Dependencies

* roscpp
* rm_common
* controller_interface
* tf2_kdl
* kdl_parser
* realtime_tools

## ROS API

#### Subscribed Topics

* **`tf`** (tf2_msgs/TFMessage)

  Obtain and manage maintenance real-time transformation information.

* **`tf_static`**(tf2_msgs/TFMessage)

  Obtain and manage maintenance static transformation information.

#### Parameters

* **`publish_rate`** (double)

  Publish frequency of state publisher, default: 50Hz.

* **`use_tf_static`** (bool)

  Set whether to use the /tf_static latched static transform broadcaster.Default: true.

* **`ignore_timestamp`** (bool)

  If true, ignore the publish_frequency and the timestamp of joint_states and publish a tf for each of the received joint_states. Default: false.

* **`buffer_duration`** (double)

  The time to keep a history of transforms.

* **`robot_description`** (string)

  The urdf xml robot description.

#### Complete description

```yaml
robot_state_controller:
  type: robot_state_controller/RobotStateController
  publish_rate: 100
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
