^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_chassis_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.11 (2023-06-20)
-------------------
* Merge pull request `#132 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/132>`_ from chenhuiYu00/change_chassis_topic
  Change chassis command topic.
* Change chassis command topic.
* Merge branch 'rm-controls:master' into master
* Merge pull request `#123 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/123>`_ from chenhuiYu00/dev/balance
  Add balance auto exit block
* Update balance model value.
* Rename BalanceMode.
* Separate balance model into functions.
* Use realtime pub in balance state.
* Merge branch 'master' into dev/balance
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/120>`_ from ye-luo-xi-tui/master
  0.1.10
* Update GYRO to RAW and enum rename.
* Merge branch 'master' into dev/balance
* Balance auto exit block add pitch limit.
* Update auto exit block.
* Add balance auto exit block.
* Merge branch 'rm-controls:master' into gpio_calibration_controller
* Contributors: 1moule, ye-luo-xi-tui, yuchen

0.1.10 (2023-03-25)
-------------------
* Merge pull request `#112 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/112>`_ from ljq-lv/Delete
  Delete the chassis mode "GYRO"
* Merge pull request `#115 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/115>`_ from Edwinlinks/fix-odom
  Add manner to fix odom2base by adding outside odometry.
* Merge pull request `#119 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/119>`_ from ye-luo-xi-tui/balance_standard
  Clear position when position_des minus position_act bigger than threshold
* Modify topic name and add comment.
* Delete the unused header.
* Add manner to fix odom2base by adding outside odometry.
* Merge branch 'rm-controls:master' into master
* Modified the chassis's README
* Delete the chassis mode "GYRO"
* Merge pull request `#111 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/111>`_ from d0h0s/master
  Updated README.md of rm_chassis_controllers.
* Repaired the example of chassis_controller.
* Added the parameters of Omni and fixed the inappropriate description.
* Updated README.md of rm_chassis_controllers.
* Merge pull request `#108 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/108>`_ from Aung-xiao/master
  add mecanum.yaml
* change the name of the controller
* add mecanum.yaml
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/106>`_ from ye-luo-xi-tui/master
  0.1.9
* Clear position when position_des minus position_act bigger than threshold.
* Contributors: Aung-xiao, Edwinlinks, QiayuanLiao, d0h0s, ljq-lv, ye-luo-xi-tui, yezi

0.1.9 (2023-02-21)
------------------
* Merge pull request `#105 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/105>`_ from ljq-lv/gimbal_toward
  Add follow gimbal's chassis control
* Add follow gimbal's chassis control
* Add follow gimbal's chassis control
* Merge pull request `#100 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/100>`_ from ye-luo-xi-tui/balance_standard
  Add balance chassis controller
* Update note.
* Update test.
* Update controller description.
* Merge branch 'master' into balance_standard
* Modify the expected location update policy.
* Take some joints' name as params.
* Fix warning.
* Add power limit.
* Compute state matrix and control matrix with given params.
* Add balance_chassis's odometry.
* Transform IMU's data to base_link.
* Try to use PID controller to fix the problem that one block's position is differ from another.
* Revised dynamics model and remove gazebo dependency.
* Merge pull request `#97 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/97>`_ from ye-luo-xi-tui/master
  0.1.8
* Fix mistake of orientation data.
* Add some test codes for balance chassis.
* Contributors: ljq-lv, ye-luo-xi-tui, yezi

0.1.8 (2022-11-24)
------------------
* Merge branch 'master' into target_velocity_correction
* Merge pull request `#89 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/89>`_ from rm-controls/dev
  Merge branch 'dev' into master
* Merge pull request `#88 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/88>`_ from NaHCO3bc/dev
  Add a push_back of joint_handles in the new OmniController.
* Add a push_back of joint_handles in the new OmniController.
* Merge pull request `#86 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/86>`_ from NaHCO3bc/Readme
  Fix the dependence part bug.
* Fix the dependence part bug.
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: NaHCO3bc, ye-luo-xi-tui, yezi

0.1.7 (2022-09-10)
------------------
* Merge pull request `#83 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/83>`_ from rm-controls/dev
  Merge the new OmniController to master
* Merge pull request `#82 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/82>`_ from NaHCO3bc/dev
  Fix some bugs in the new OmniController.
* Compute the params and fix some bugs.
* Rename the function forwardKinematics to odometry.
* Merge pull request `#80 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/80>`_ from qiayuanliao/master
  New and elegant OmniController
* Fix bug in the new OmniController
* Add a new and elegant OmniController
* Merge remote-tracking branch 'origin/master'
* Contributors: NaHCO3bc, QiayuanLiao, qiayuan

0.1.6 (2022-06-16)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

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
