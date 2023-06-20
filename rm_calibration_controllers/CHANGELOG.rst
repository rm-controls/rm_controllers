^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_calibration_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.11 (2023-06-20)
-------------------
* Merge pull request `#127 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/127>`_ from 1moule/gpio_calibration_controller
  Rewrite the stopping function and set calibration success to false in stopping function
* Rewrite the stopping function and set calibration success to false in the stopping function.
* Merge pull request `#126 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/126>`_ from 1moule/master
  Remove unnecessary variables
* Remove unnecessary variables.
* Merge pull request `#125 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/125>`_ from rm-controls/calibration
  Add gpio calibration controller
* Merge pull request `#116 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/116>`_ from 1moule/gpio_calibration_controller
  Split the calibration controller and add a controller that uses gpio calibration
* Modefy CMakeLists, delete TODO and initialize a variable.
* Modified to get the error message when gpio is obtained.
* Delete vector and some unnecessary code.
* Merge branch 'master' into dev/balance
* Change gpio calibration controller to the same one and modify gpio calibration logic.
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/120>`_ from ye-luo-xi-tui/master
  0.1.10
* Modify the unmodified name in the joint calibration controller.
* Rename joint calibration controller.
* Write the velocity threshold in the base class.
* Modify the name of an enumeration type.
* Modify the name of an enumeration type.
* Modify variable name.
* Delete some comments, modify the initialization function of the gpio calibration controller base class.
* Modified hardware interface for instantiating template classes.
* Add a new line at the end and delete update function of calibration_base.h file.
* Factor out the calibration controller into a form derived from a base class and modify the controller appropriately.
* Use gpio handle to replace gpio call back function.
* Solved the problem of not being in the detection range of the hall switch when starting the calibration.
* Modify gpio calibration controller scheme to first use speed control to find a fixed point, and then use position control to reach.
* Modify logic and callback function of gpio calibration controller.
* Modify queue length of gpio subscriber.
* Merge branch 'rm-controls:master' into gpio_calibration_controller
* Add gpio calibration controller.
* Contributors: 1moule, ye-luo-xi-tui, yuchen

0.1.10 (2023-03-25)
-------------------
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/106>`_ from ye-luo-xi-tui/master
  0.1.9
* Contributors: ye-luo-xi-tui

0.1.9 (2023-02-21)
------------------
* Merge pull request `#103 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/103>`_ from L-SY/fix_return
  Fix calibration controller no return true.
* Fix bug.
* Run pre-commit.
* Merge branch 'master' into balance_standard
* Add engineer trigger ui .
* Merge pull request `#96 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/96>`_ from L-SY/fix_return
  Fix joint_calibration_controller
* Delete repetitive set.
* delete useless target_velocity\_.
* Merge pull request `#97 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/97>`_ from ye-luo-xi-tui/master
  0.1.8
* eyes is lost.
* Update fix can not return .
* Contributors: lsy, ye-luo-xi-tui, yezi

0.1.8 (2022-11-24)
------------------
* Merge branch 'master' into target_velocity_correction
* Merge pull request `#87 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/87>`_ from NaHCO3bc/Readme
  Fix the dependence bug in rm_calibration_controllers.
* Fix the dependence bug in rm_calibration_controllers.
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: NaHCO3bc, ye-luo-xi-tui, yezi

0.1.7 (2022-09-10)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

0.1.6 (2022-06-16)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

0.1.5 (2022-06-10)
------------------
* Merge remote-tracking branch 'origin/master'
* 0.1.4
* Merge branch 'master' into gimbal_track
* Merge pull request `#45 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/45>`_ from mlione/master
  Delete some config files in rm_controllers.
* Delete some config files in rm_controller.
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Contributors: QiayuanLiao, YuuinIH, mlione, qiayuan, yezi

0.1.3 (2022-03-28)
------------------
* Merge branch 'master' into forward_feed
* Merge branch 'master' into standard3
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge branch 'master' into chassis/fix_filter
* Merge remote-tracking branch 'origin/master'
* Merge branch 'namespace'
  # Conflicts:
  #	rm_chassis_controllers/README.md
* Merge pull request `#15 <https://github.com/rm-controls/rm_controllers/issues/15>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control
* Change name of namespace:from hardware_interface to rm_control.
* Merge pull request `#7 <https://github.com/rm-controls/rm_controllers/issues/7>`_ from kbxkgxjg/patch-1
  Add white line behind title on calibration_controller  README.md
* Add white line behind title
* Rename motor to actuators
* Merge pull request `#6 <https://github.com/rm-controls/rm_controllers/issues/6>`_ from kbxkgxjg/master
  Add doxygen comments on joint_calibration_controller.h
* Modify the meaning of `param req`
* Fix clang-format error
* Modify the params
* Update the comments of is calibrated()
* Update the comments of starting()
* Update the comments of update()
* Update the comments of init()
* Modify 'current joint state' to 'current calibration joint state'
* Modift '\' to '@'
* Modify "actuator state" to "joint state " and Modify motors to actuators
* Modify format
* Delete white line
* Delete a whitespace
* Add annotation to function
* Use “pragma once” in rm_calibration_controllers headers instead of include guards.
* Merge pull request `#2 <https://github.com/rm-controls/rm_controllers/issues/2>`_ from kbxkgxjg/master
  Update README.md on rm_calibration_controllers
* Modify the meaning of `search_velocity` and `threshold`
* Revert "Modify 'is_calibrated_srv' to 'is_calibrated'"
  This reverts commit b5b06dfe
* Delete a net
* Modify 'is_calibrated_srv' to 'is_calibrated'
* Add angular in front of velocity
* fixed grammar error
* Modify the meaning of `is_calibrated_srv\_`
* Modify a wrong hardware interface name
* Add "Hardware interface type", modify "License" and Delete number
* Modify actuator to motor
* Add a white line in front of  “Or better”
* Delete 啊
* Delete a white line
* Modify the font in Installation
* Modify overview
* Update the format of Installation
* Add . behind CLIBRATIED
* Modify the meaning of `search_velocity` `threshold`
* Delete a white line
* Add tap before begin and modify Installation from Packages
* Delete whitespace before 'When ....'
* Explain the 'search_velocity' 'threshold' together
* Modify services to service
* Modify format and modify the explanation of `is_calibrated_srv\_`
* Delete a whitespace between parameters and data type
* Modify the font size in ROS API, and  delete ' . ' behind 3 .1 3.2
* Add a whitespace between parameter and date type
* Add '  ' to double
* Modify Installation from Packages
* Modify date type of 'is_calibrated_srv\_'
* Update 'Installation from Packages'
* Update the explanation of 'is_calibrated_srv\_'
* Delete a whitespace
* Delete '*' and change number
* Delete a whitespace
* Delete usage, and change the data type of `is_calibrated_srv\_`
* Delete config and pid
* Add the type of the data And delete 'type' 'joint' 'actuators' parameters
* Add rm_calibration_controllers README.md
* Update README.md
  Add a line
* Delete a whitespace
* Add README.md
* Fix wrong naming "include/rm_gimbal_controller"
* Run pre-commit
* Contributors: BruceLannn, QiayuanLiao, chenzheng, kbxkgxjg, ljq, qiayuan, yezi, 吕骏骐

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to rm_calibration_controllers source files
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to rm_calibration_controllers/rm_calibration_controllers, prepare for merge
* Contributors: qiayuan
