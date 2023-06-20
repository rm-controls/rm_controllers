^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_shooter_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.11 (2023-06-20)
-------------------
* Merge pull request `#129 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/129>`_ from ye-luo-xi-tui/master
  Shooter_controller would not check block when friction wheel don't rotate
* Shooter_controller would not check block when friction wheel don't rotate.
* Merge branch 'master' into dev/balance
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/120>`_ from ye-luo-xi-tui/master
  0.1.10
* Merge branch 'rm-controls:master' into gpio_calibration_controller
* Contributors: 1moule, ye-luo-xi-tui, yezi, yuchen

0.1.10 (2023-03-25)
-------------------
* Merge pull request `#118 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/118>`_ from ye-luo-xi-tui/master
  Publish shoot state
* Publish shoot state.
* Merge pull request `#109 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/109>`_ from ye-luo-xi-tui/master
  Modifier default value of forward_push_threshold and exit_push_threshold
* Modifier default value of forward_push_threshold and exit_push_threshold.
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/106>`_ from ye-luo-xi-tui/master
  0.1.9
* Contributors: ye-luo-xi-tui, yezi

0.1.9 (2023-02-21)
------------------
* Merge branch 'master' into balance_standard
* Merge remote-tracking branch 'origin/fix_return' into fix_return
* Merge pull request `#97 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/97>`_ from ye-luo-xi-tui/master
  0.1.8
* Merge branch 'rm-controls:master' into fix_return
* Contributors: L-SY, lsy, ye-luo-xi-tui, yezi

0.1.8 (2022-11-24)
------------------
* Merge pull request `#93 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/93>`_ from NaHCO3bc/master
  Fix the bug that the shooter cannot be turned from push to ready
* Modify the name and the description of the params about push forward threshold.
* Optimize the logic of entering the block mode.
* Fix the bug that shooter cannot push or enter block when the position error is too big.
* Modify the params name.
* Modify the params about enter and exit push mode.
* Parametric position difference of trigger.
* Fix the bug that the shooter cannot be turned from push to ready.
* Merge pull request `#86 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/86>`_ from NaHCO3bc/Readme
  Fix the dependence part bug.
* Modify the test file folder.
* Fix the dependence part bug.
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: NaHCO3bc, ye-luo-xi-tui

0.1.7 (2022-09-10)
------------------
* Try two fix double shoot
* Merge remote-tracking branch 'origin/master'
* Fix bug of shooting if statement
* Contributors: qiayuan

0.1.6 (2022-06-16)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

0.1.5 (2022-06-10)
------------------
* Add hz counting in trigger test mode.
* Merge remote-tracking branch 'origin/master'
* Add testing option to shooter for testing the trigger without friction wheel
* 0.1.4
* Merge branch 'master' into gimbal_track
* Merge pull request `#45 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/45>`_ from mlione/master
  Delete some config files in rm_controllers.
* Delete some config files in rm_controller.
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, mlione, qiayuan, yezi

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
* Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into standard3
* Update standard3 config
* Merge remote-tracking branch 'origin/master'
* Contributors: QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Update the config of rm_shooter_controller, load only one controller on launch instead of spawn controllers
* Merge branch 'master' into chassis/fix_filter
* Merge remote-tracking branch 'origin/master'
* Merge branch 'namespace'
  # Conflicts:
  #	rm_chassis_controllers/README.md
* Merge pull request `#15 <https://github.com/rm-controls/rm_controllers/issues/15>`_ from ye-luo-xi-tui/namespace
  Change name of namespace:from hardware_interface to rm_control
* Change name of namespace:from hardware_interface to rm_control.
* Merge pull request `#8 <https://github.com/rm-controls/rm_controllers/issues/8>`_ from Edwinlinks/master
  Update README.md in rm_shooter_controller
* Update the format of Bugs & Feature Requests README.md in  rm_shooter_controller
* Fix LICENSE url
* Update the format of Bugs & Feature Requests README.md in rm_shooter_controller
* Update the README.md in rm_shooter_controller
* Use “pragma once” in rm_shooter_controllers headers instead of include guards.
* Correct dependencies and interface type.
* Correct some shooter param's description.
* Simplify shooter README.md.
* Update shooter param's description.
* Correct readme format.
* Correct readme format.
* Update controllers README.
* Merge remote-tracking branch 'origin/master'
* Update README.md
* Delet a space bar.
  Delet space bar after sudo rosdep install --from-paths src
* Add some param's description
* Update shooter controller's README.md
* Fix wrong naming "include/rm_shooter_controller"
* Run pre-commit
* Format rm_shooter_controllers using clang-format
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, chenzheng, link Edwin, qiayuan, yezi, 沐

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to rm_chassis_controllers and rm_gimbal_controllers source files
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to rm_shooter_controllers/rm_shooter_controllers, prepare for merge
* Contributors: qiayuan
