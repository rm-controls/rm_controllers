^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_gimbal_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.11 (2023-06-20)
-------------------
* Merge pull request `#135 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/135>`_ from ye-luo-xi-tui/feedforward
  Add input feedforward and fix a bug in computing desire vel at TRACK mode
* Add input feedforward and fix a bug in computing desire vel at TRACK mode.
* Merge pull request `#124 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/124>`_ from ye-luo-xi-tui/resistance_compensation
  Add velocity_dead_zone and effort_dead_zone
* Merge branch 'master' into dev/balance
* Add velocity_dead_zone and effort_dead_zone.
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/120>`_ from ye-luo-xi-tui/master
  0.1.10
* Merge branch 'rm-controls:master' into gpio_calibration_controller
* Contributors: 1moule, ye-luo-xi-tui, yezi, yuchen

0.1.10 (2023-03-25)
-------------------
* Merge branch 'rm-controls:master' into master
* Merge pull request `#114 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/114>`_ from ye-luo-xi-tui/resistance_compensation
  Add resistance compensation on yaw
* Add resistance compensation on yaw.
* Merge pull request `#113 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/113>`_ from ye-luo-xi-tui/master
  Use Vector3WithFilter in rm_common instead
* Use Vector3WithFilter in rm_common instead.
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/106>`_ from ye-luo-xi-tui/master
  0.1.9
* Contributors: ye-luo-xi-tui, yezi

0.1.9 (2023-02-21)
------------------
* Merge pull request `#104 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/104>`_ from ye-luo-xi-tui/balance_standard
  Fix bug.
* Fix bug.
* Merge pull request `#102 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/102>`_ from ljq-lv/fix_warning
  Fixed the bug of gimbal warning
* Add the else to judge mode TRACK
* Fixed the bug of gimbal warning
* Merge branch 'master' into balance_standard
* Merge pull request `#97 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/97>`_ from ye-luo-xi-tui/master
  0.1.8
* Contributors: ljq-lv, ye-luo-xi-tui, yezi

0.1.8 (2022-11-24)
------------------
* Merge pull request `#92 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/92>`_ from ye-luo-xi-tui/target_velocity_correction
  Estimate chassis vel with moving average and subtract chassis_vel from target_vel
* Fix trylock bug.
* Subtract chassis_vel from target_vel.
* Estimate chassis vel with moving average.
* Merge pull request `#86 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/86>`_ from NaHCO3bc/Readme
  Fix the dependence part bug.
* Modify the test file folder.
* Fix the dependence part bug.
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: NaHCO3bc, ye-luo-xi-tui, yezi

0.1.7 (2022-09-10)
------------------
* Optimize TRACK mode of rm_gimbal_controller.
* Change frame "map" to "odom".
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan, yezi

0.1.6 (2022-06-16)
------------------
* Merge branch 'gimbal/chassis_vel'
* Merge pull request `#77 <https://github.com/rm-controls/rm_controllers/issues/77>`_ from ye-luo-xi-tui/gimbal/chassis_vel
  Fix a stupid bug
* Fix a stupid bug.
* Add chassis_vel as a feedforward to the yaw joint
* Merge remote-tracking branch 'origin/master'
* Add chassis_vel to yaw's PID
* Contributors: QiayuanLiao, qiayuan, yezi

0.1.5 (2022-06-10)
------------------
* Merge pull request `#71 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/71>`_ from ye-luo-xi-tui/gimbal/joint_velocity
  Set joint desired velocity according to the target velocity
* Fix a bug in setting joint desired velocity.
* Set joint desired velocity according to the target velocity.
* Merge pull request `#69 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/69>`_ from ChenZheng29/master
  Correct track topic message type:TrackData
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#66 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/66>`_ from ye-luo-xi-tui/master
  Use motors' data in pid controller when imu_name isn't set
* Use motors' data in pid controller when imu_name isn't set.
* Correct the message type of gimbal subscription '/track' topic from 'rm_msgs::TrackCmd' to 'rm_msgs::TrackData'
* Modify the track topic name and message, and unify the track interface
* Merge remote-tracking branch 'origin/master'
* 0.1.4
* Move all computation in gimbal_controller i.r.t. "odom" instead of "map"
* Merge pull request `#52 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/52>`_ from ye-luo-xi-tui/gimbal_track
  Update gimbal_controller
* Modifier TrackCmd.msg format.
* Improve logic of function Controller::setDes().
* Fix a frame error in track mode.
* Merge branch 'master' into gimbal_track
* Merge pull request `#45 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/45>`_ from mlione/master
  Delete some config files in rm_controllers.
* Delete some config files in rm_controller.
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Merge branch 'master' into gimbal_track
* Rename a msg.
* Modifier track subscribe topic
* Contributors: QiayuanLiao, YuuinIH, chenzheng, mlione, qiayuan, yezi

0.1.3 (2022-03-28)
------------------
* Merge pull request `#42 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/42>`_ from ye-luo-xi-tui/fix_gimbal
  Fix bug in gimbal_controller
* Merge pull request `#37 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/37>`_ from ye-luo-xi-tui/forward_feed
  Add feedforward in gimbal control
* Merge branch 'master' into forward_feed
* Simplify the codes. Set vel_target under rate mode.
* Fix bug which relative to limit in gimbal_controller.
* Merge pull request `#40 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/40>`_ from ye-luo-xi-tui/maintain
  Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into 'standard3'.
* Merge branch 'master' into maintain
  # Conflicts:
  #	rm_chassis_controllers/config/standard3.yaml
  #	rm_chassis_controllers/config/standard4.yaml
* Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into standard3
* Delete eigen, tf2_eigen instead.
* chore: add missing deps
* Merge remote-tracking branch 'origin/master'
* Change frame id of gimbal while transforming angular_vel form imu to pitch/yaw for engineer or sentry.
* Add feedforward in gimbal control.
* Contributors: QiayuanLiao, StarHeart, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge pull request `#30 <https://github.com/rm-controls/rm_controllers/issues/30>`_ from ljq-lv/rm_gimbal_controllers
  Modify namespace on rm_gimbal_controllers
* Modify namespace from bullet_solver to rm_gimbal_controllers
* Merge branch 'master' into omni_wheel_controller
* Merge remote-tracking branch 'origin/master'
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Merge pull request `#17 <https://github.com/rm-controls/rm_controllers/issues/17>`_ from ChenZheng29/master
  Fix the abnormal gimbal caused by the different representation of angle between TF and URDF
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Add the judgment of the pitch of the gimbal
* Fix gimbal position limit
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Update the config of rm_gimbal_controllers, load only one controller on launch instead of spawn controllers
* Merge branch 'gimbal/opti_or_simplify' into imu_filter_controllers
* Test imu2can with gimbal, fix a stupid bug
* Rename standard to gimbal_base
* Correct format.
* Merge branch 'master' into chassis/fix_filter
* Merge remote-tracking branch 'origin/master'
* Update static_transform_publisher from tf to tf2
* Remove updateTf() of rm_gimbal_controllers
* Fix tf time of rm_gimbal_controllers
* Use gyro data as gimbal joint velocity.
* Sort code, add imu_sensor_interface
* Simplify rm_gimbal_controllers and tested on gazebo
* Modified GimbalCmd.msg, and delete moving_average_filter
* Merge branch 'namespace'
  # Conflicts:
  #	rm_chassis_controllers/README.md
* Merge pull request `#15 <https://github.com/rm-controls/rm_controllers/issues/15>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control
* Correct format
* Change name of namespace:from hardware_interface to rm_control.
* Merge pull request `#5 <https://github.com/rm-controls/rm_controllers/issues/5>`_ from BruceLannn/master
  Reformat gimbal controllers' README.md
* Update publish rate description.
* Update the command of installing shooter controller.
* Update publish rate description.
* Correct GimbanlCmd to GimbalCmd and delet ##cfg
* Update model_desire topic description.
* Correct a format error.
* Add model_desire and model_real description in the published topic.
* Update cfg file description.
* Correct param type format.
* Update moving average filter's param.
* Update some param's description.
* Supplementary unit of center_offset_z.
* Update parameter's description.
* Use “pragma once” in rm_gimbal_controllers headers instead of include guards.
* Update Overview's keywords.
* Update Overview.
* Reformat README.md
* Update shooter param's description.
* Correct readme format.
* Correct readme format.
* Correct readme format.
* Update controllers README.
* Update controllers README.
* Fix wrong naming "include/rm_gimbal_controller"
* Run pre-commit
* Code style
* Format rm_gimbal_controllers using clang-format
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, chenzheng, kbxkgxjg, qiayuan, ye-luo-xi-tui, yezi

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to rm_chassis_controllers and rm_gimbal_controllers source files
* Add add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to rm_gimbal_controllers/rm_gimbal_controllers, prepare for merge
* Contributors: qiayuan
