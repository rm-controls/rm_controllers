^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2022-03-28)
------------------
* Merge branch 'master' into forward_feed
* Merge branch 'master' into standard3
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Merge branch 'master' into chassis/balance_imu_interface
* Clear tf buffer when simulation reset
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Add the config of robot_state_controller, load only one controller on launch instead of spawn controllers
* Merge branch 'master' into chassis/fix_filter
* Merge remote-tracking branch 'origin/master'
* Fix tf_transform cleared before publish
* Check the timestamp when receiving the tf transforms outside the rm_hw/rm_gazebo
* Real time safe for robot_state_controller
* Merge branch 'namespace'
  # Conflicts:
  #	rm_chassis_controllers/README.md
* Merge pull request `#15 <https://github.com/rm-controls/rm_controllers/issues/15>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control
* Change name of namespace:from hardware_interface to rm_control.
* Remove README.md
* Use “pragma once” in robot_state_controller headers instead of include guards.
* Fix "robot_state_controller: CMakeLists.txt: error: unconfigured build_depend on 'pluginlib'"
* Format robot_state_controller using clang-format
* Contributors: BruceLannn, MuZhou233, QiayuanLiao, YuuinIH, chenzheng, qiayuan, yezi

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to robot_state_controller source files
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to robot_state_controller/robot_state_controller, prepare for merge
* Contributors: qiayuan
