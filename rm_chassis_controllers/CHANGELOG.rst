^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_chassis_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
