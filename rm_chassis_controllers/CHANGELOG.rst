^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_chassis_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2022-06-10)
------------------
* Update frames in update function.
* Add command_source_frame to follow mode.
* Merge remote-tracking branch 'origin/master'
* 0.1.4
* Add damping behavior to ReactionWheelController
* Add recover behavior to ReactionWheelController
* Add a low-pass filter to eliminate the constant pitch angle offset to ReactionWheelController
* Add a naive method for determining orientation of balance chassis
* Use new reaction wheel state space model
* Add setZero to Q and R
* Merge remote-tracking branch 'origin/master'
* Remove "convert model from continuous to discrete time" from ReactionWheel, the rm_common::Lqr accept continuous model.
* Merge pull request `#57 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/57>`_ from NaHCO3bc/master
  Update ReactionWheelController
* Change the number of joint name reads,change the QR matrix to an array and update the rm_chassis_controllers_plugins.xml.
* Merge branch 'chassis/reaction_wheel'
* Merge pull request `#56 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/56>`_ from NaHCO3bc/chassis/reaction_wheel
  Update ReactionWheelController
* Update reaction_wheel.cpp and add template_reaction_wheel.yaml
* Add some comments of ReactionWheelController
* Remove BalanceController
* Add ReactionWheelController
* Merge pull request `#51 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/51>`_ from Edwinlinks/sentry_catapult
  Complete sentry catapult and delete the redundant code
* Merge branch 'master' into gimbal_track
* Complete sentry catapult and delete the redundant code
* Merge pull request `#45 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/45>`_ from mlione/master
  Delete some config files in rm_controllers.
* Merge pull request `#50 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/50>`_ from ye-luo-xi-tui/ori
  Make rm_orientation_controller publish tf use imu data on the topic
* Delete some config files in rm_controller.
* Add a param publish_odom_tf, it decided whether tf would be published.
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Merge branch 'master' into gimbal_track
* Contributors: Edwinlinks, QiayuanLiao, YuuinIH, mlione, nahco3bc, qiayuan, yezi

0.1.3 (2022-03-28)
------------------
* Merge branch 'master' into forward_feed
* Merge pull request `#40 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/40>`_ from ye-luo-xi-tui/maintain
  Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into maintain
  # Conflicts:
  #	rm_chassis_controllers/config/standard3.yaml
* Merge pull request `#41 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/41>`_ from ye-luo-xi-tui/standard3
  Update standard3 config
* Merge branch 'master' into 'standard3'.
* Merge branch 'master' into maintain
  # Conflicts:
  #	rm_chassis_controllers/config/standard3.yaml
  #	rm_chassis_controllers/config/standard4.yaml
* Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into standard3
* Add missing parameters and format rm_chassis_controllers
* Update standard3 config
* Merge remote-tracking branch 'origin/master'
* Update standard3 chassis_controller config.
* Contributors: QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge pull request `#31 <https://github.com/rm-controls/rm_controllers/issues/31>`_ from ye-luo-xi-tui/modify_params
  Modify chassis configuration
* Update standard3.yaml.
* Update sentry chassis config.
* Update standard5.yaml
* Merge pull request `#27 <https://github.com/rm-controls/rm_controllers/issues/27>`_ from ye-luo-xi-tui/omni_wheel_controller
  Add omni wheel controller
* Fix bug in kinematics.
* Change param name.
* Merge branch 'master' into omni_wheel_controller
* Merge remote-tracking branch 'origin/master'
* Merge branch 'rm-controls:master' into master
* Merge pull request `#24 <https://github.com/rm-controls/rm_controllers/issues/24>`_ from ye-luo-xi-tui/fix_power_limit
  Fix a bug in power limit.
* Add OmniController.
* Fix a bug in power limit.
  (cherry picked from commit 81d1e880ea67aed189cb762819f63d4a5fba6b9b)
* Merge remote-tracking branch 'origin/master'
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Merge pull request `#19 <https://github.com/rm-controls/rm_controllers/issues/19>`_ from ye-luo-xi-tui/new_power_limit
  New power limit
* Add power_offset in new power limit
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Optimize the code.
* New power limit
* Format
* Use imu_sensor_interface in BalanceController
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Code style
* Set odom tf in each update of rm_chassis_controllers
* Merge branch 'master' into chassis/balance_imu_interface
* Run pre-commit
* Modify ChassisBase to template class prepare for adding imu_sensor_interface(only BalanceChassis)
* Update the config of imu_chassis_controllers, load only one controller on launch instead of spawn controllers
* Fix pre-commit
* Correct code format.
* Correct code format.
* Correct format.
* Merge remote-tracking branch 'origin/chassis/fix_filter' into chassis/fix_filter
  # Conflicts:
  #	rm_chassis_controllers/src/chassis_base.cpp
  #	rm_chassis_controllers/src/swerve.cpp
* Filter the linear vel before transform and filter the angular vel after PID.
* Delete if (std::abs(vel_cmd\_.x) + std::abs(vel_cmd\_.y) >= 0.01)
* Merge branch 'master' into chassis/fix_filter
* Delet #endif
* Merge remote-tracking branch 'origin/master'
* Filter the linear vel before transform and filter the angular vel after PID.
* Set transform on buffer when publishing odom tf.
* Update static_transform_publisher from tf to tf2
* Merge branch 'namespace'
  # Conflicts:
  #	rm_chassis_controllers/README.md
* Merge pull request `#15 <https://github.com/rm-controls/rm_controllers/issues/15>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control
* Merge pull request `#10 <https://github.com/rm-controls/rm_controllers/issues/10>`_ from ye-luo-xi-tui/master
  Update README of chassis controller
* Change name of namespace:from hardware_interface to rm_control.
* Update README.md of rm_chassis_controllers
* Fix format error
* Add doxygen on sentry.h
* Add doxygen on chassis_base.h
* Add doxygen on mecanum.h
* Add doxygen on balance.h
* Add nav_msgs to rm_chassis_controllers
* update README of chassis controller
* update README of chassis controller
* Update README.md
* Merge pull request `#9 <https://github.com/rm-controls/rm_controllers/issues/9>`_ from ye-luo-xi-tui/master
  update README.md of chassis controller
* README.md
* Code style
* Use “pragma once” in rm_chassis_controllers headers instead of include guards.
* Update shooter param's description.
* Correct readme format.
* Correct readme format.
* Correct readme format.
* Update controllers README.
* Update controllers README.
* Run pre-commit
* Format rm_chassis_controllers using clang-format
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, chenzheng, kbxkgxjg, qiayuan, ye-luo-xi-tui, yezi

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to rm_chassis_controllers and rm_gimbal_controllers source files
* Remove test_depend of rm_chassis_controllers
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to rm_chassis_controllers/rm_chassis_controllers, prepare for merge
* Contributors: qiayuan
