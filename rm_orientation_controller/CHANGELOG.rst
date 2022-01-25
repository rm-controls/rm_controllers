^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_orientation_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
