^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_orientation_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.11 (2023-06-20)
-------------------
* Merge branch 'master' into dev/balance
* Merge pull request `#120 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/120>`_ from ye-luo-xi-tui/master
  0.1.10
* Merge branch 'rm-controls:master' into gpio_calibration_controller
* Contributors: 1moule, ye-luo-xi-tui, yuchen

0.1.10 (2023-03-25)
-------------------
* Merge pull request `#106 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/106>`_ from ye-luo-xi-tui/master
  0.1.9
* Contributors: ye-luo-xi-tui

0.1.9 (2023-02-21)
------------------
* Merge branch 'master' into balance_standard
* Merge pull request `#97 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/97>`_ from ye-luo-xi-tui/master
  0.1.8
* Contributors: ye-luo-xi-tui, yezi

0.1.8 (2022-11-24)
------------------
* Merge pull request `#91 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/91>`_ from ye-luo-xi-tui/imu_offline
  Dealing with the situation of imu offline.
* Dealing with the situation of imu offline.
* Merge pull request `#86 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/86>`_ from NaHCO3bc/Readme
  Fix the dependence part bug.
* Modify the test file folder.
* Fix the dependence part bug.
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: NaHCO3bc, ye-luo-xi-tui, yezi

0.1.7 (2022-09-10)
------------------
* Optimize rm_orientation_controller.
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan, yezi

0.1.6 (2022-06-16)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

0.1.5 (2022-06-10)
------------------
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#63 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/63>`_ from ye-luo-xi-tui/master
  Publish tf from odom to base_link use imu data which is got from interface until getting imu data from topic
* 0.1.4
* Publish tf from odom to base_link use imu data which is got from interface until get imu data from topic.
* Merge pull request `#53 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/53>`_ from ye-luo-xi-tui/orientation_controller
  Fix a bug that time stamp error
* Fix a bug that time stamp error when it can't find transform from imu to odom.
* Merge branch 'master' into gimbal_track
* Merge pull request `#45 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/45>`_ from mlione/master
  Delete some config files in rm_controllers.
* Merge pull request `#50 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/50>`_ from ye-luo-xi-tui/ori
  Make rm_orientation_controller publish tf use imu data on the topic
* Delete some config files in rm_controller.
* Solve a problem that look up future transform.
* Delete publish_rate and relative codes.
* Publish tf use imu data on the topic.
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Contributors: QiayuanLiao, YuuinIH, mlione, qiayuan, yezi

0.1.3 (2022-03-28)
------------------
* Merge branch 'master' into forward_feed
* Merge pull request `#40 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/40>`_ from ye-luo-xi-tui/maintain
  Delete configuration of robot_state_controller in each of controllers' config file
* Delete configuration of robot_state_controller in each of controllers' config file
* Merge branch 'master' into standard3
* Merge remote-tracking branch 'origin/master'
* Contributors: QiayuanLiao, qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------
* Merge branch 'master' into omni_wheel_controller
* Merge pull request `#23 <https://github.com/rm-controls/rm_controllers/issues/23>`_ from ye-luo-xi-tui/master
  Add ImuFilterBase, overwrite ComplementaryController, add MadgwickController
* Fix format error
  (cherry picked from commit f7519e6800095d933f9d4c85f108ae5260001572)
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Run pre-commit
* Merge branch 'master' into gimbal/opti_or_simplify
  # Conflicts:
  #	rm_orientation_controller/launch/load_controllers.launch
  #	rm_orientation_controller/src/orientation_controller.cpp
* Merge branch 'master' into chassis/balance_imu_interface
* Run pre-commit
* Update the config of rm_orientation_controller, load only one controller on launch instead of spawn controllers
* Correct "rm_orientation_controllers" to "rm_orientation_controller".
* Fix pre-commit
* Code style and delete unnecessary
* Merge branch 'master' into imu_filter_controllers
  # Conflicts:
  #	imu_filter_controllers/CMakeLists.txt
  #	imu_filter_controllers/include/imu_filter_controllers/complementary_controller.h
  #	imu_filter_controllers/package.xml
  #	imu_filter_controllers/src/complementary_controller.cpp
* Upgrade "hardware_interface::RobotStateInterface" to "rm_control::RobotStateInterface".
* Update orientation controller version.
* Merge remote-tracking branch 'alias_memory/metapackage'
* Move all files to rm_orientation_controller/rm_orientation_controller,prepare for merge
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, qiayuan, yezi
