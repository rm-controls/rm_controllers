# rm_gimbal_controllers
***
## Overview
***
This is a package about the cloud platform control,including the bullet and moving controll.
#### Key worlds:
gimbal,bullet,ROS
### License
The source code is released under a [ BSD 3-Clause license](http://192.168.0.100:7070/dynamicx/rm_gimbal_controllers/-/blob/master/LICENSE)
#### Author:QiayuanLiao
#### Affiliation:DynamicX
#### Maintainer: QiayuanLiao
The PACKAGE NAME package has been tested under [ROS](https://www.ros.org/) Indigo, Melodic and Noetic on respectively Ubuntu 14.04, 18.04 and
20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

![Example image](doc/example.jpg)

[comment]: <> (### Publications)

[comment]: <> (If you use this work in an academic context, please cite the following publication&#40;s&#41;:)

[comment]: <> (* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference)

[comment]: <> (  on Intelligent Robots and Systems &#40;IROS&#41;, 2015. &#40;[PDF]&#40;http://dx.doi.org/10.3929/ethz-a-010173654&#41;&#41;)

[comment]: <> (        @inproceedings{Fankhauser2015,)

[comment]: <> (            author = {Fankhauser, P\'{e}ter and Hutter, Marco},)

[comment]: <> (            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems &#40;IROS&#41;},)

[comment]: <> (            title = {{PAPER TITLE}},)

[comment]: <> (            publisher = {IEEE},)

[comment]: <> (            year = {2015})

[comment]: <> (        })

## Installation
***
#### Installation from Packages
        sudo apt-get install ros-noetic-...
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

  sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone http://frp.acmetech.top:7070/dynamicx/rm_gimbal_controllers.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch ros_package_template ros_package_template.launch


## Config
***
* engineer.yaml
* hero.yaml
* sentry.yaml
* standard3.yaml
* standard4.yaml
* standard5.yaml
## Launch files
***
* load_controller.launch
## Bugs & Feature Requests
***
Please report bugs and request features using the [Issue Tracker
](https://github.com/gdut-dynamic-x/rm_template/issues).