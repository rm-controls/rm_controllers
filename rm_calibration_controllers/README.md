# rm_calibration_controllers

## Overview

Since the zero point of some motors will change after power off, rm_calibration_controllers will move after it is started until motors reach the position we wanted,and motors position will be reset to zero,which can come true by two ways.One is that mechanical_calibration_controller stops moving after it reaches the mechanical limit.Another one is that gpio_calibration_controller moves at a fast speed until gpio changes to be different from initial gpio.Then it will retreat at a small angle and restore initial gpio.Finally it moves at a slow speed until the state of gpio changes to be different from initial gpio again.

**Keywords:** calibration, ROS, position.

### License

The source code is released under a [ BSD 3-Clause license](https://github.com/rm-controls/rm_controllers/blob/master/rm_calibration_controllers/LICENSE).

**Author: DynamicX<br />
Affiliation: DynamicX<br />
Maintainer: DynamicX**

The package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 18.04 and 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Hardware interface type

+ `EffortJointInterface` Used to send effort command to target joint to make it reach the calibration speed.
+ `ActuatorExtraInterface` Used to obtain the information of the target actuators offset, current position, the state of the whether it is stopped and the state of whether it is calibrated.
+ `GpioStateInterface` Used to obtain the state of gpio.


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-calibration-controllers

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Dependencies

* roscpp
* rm_common
* effort_controllers
* controller_interface

## ROS API

### Service

* **`is_calibrated`** ([control_msgs/QueryCalibrationState](http://docs.ros.org/en/api/control_msgs/html/srv/QueryCalibrationState.html))

  When requesting to this server, it will return response about whether target joint has been calibrated.


### Parameters

#### calibration_base

* **`search_velocity`** (double)

  The joint velocity of calibrating.

#### mechanical_calibration_controller

* **`velocity_threshold_`** (double)

  This is velocity `threshold`. When the real time velocity of target joint lower than threshold, and last for a while,
  it can switch CALIBRATED from MOVING.

#### gpio_calibration_controller

* **`backward_angle`** (double)

  The angle of retreat when gpio changes to be different form initial gpio for the first time.

* **`slow_forward_velocity`** (double)

  The velocity of second forward movement for reaching a more accurate calibration-position.

* **`pos_threshold`** (double)

  The threshold for the difference between the command position and the current position.

### Complete description

#### mechanical_calibration_controller

```yaml
trigger_calibration_controller:
  type: rm_calibration_controllers/MechanicalCalibrationController
  actuator: [ trigger_joint_motor ]
  velocity:
    search_velocity: 4.0
    vel_threshold: 0.001
    joint: trigger_joint
    pid: { p: 0.8, i: 0, d: 0.0, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true }
```

#### gpio_calibration_controller
```yaml
yaw_calibration_controller:
  type: rm_calibration_controllers/GpioCalibrationController
  actuator: [ yaw_joint_motor ]
  gpio: "yaw"
  initial_gpio_state: false
  velocity:
    search_velocity: -4.0
    slow_forward_velocity: -2.0
    joint: yaw_joint
    pid: { p: 0.19, i: 0, d: 0.0, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true }
  position:
    pos_threshold: 0.01
    backward_angle: -0.15
    joint: yaw_joint
    pid: { p: 7.0, i: 0, d: 0.0, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true }
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/rm-controls/rm_controllers/issues).
