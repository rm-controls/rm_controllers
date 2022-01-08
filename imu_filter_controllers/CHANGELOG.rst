^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_filter_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2022-01-08)
------------------
* Merge branch 'master' into omni_wheel_controller
* Merge pull request `#23 <https://github.com/rm-controls/rm_controllers/issues/23>`_ from ye-luo-xi-tui/master
  Add ImuFilterBase, overwrite ComplementaryController, add MadgwickController
* Adjust some params and merge from source repository.
* Add arg to launch file
* Fix error
* Merge remote-tracking branch 'origin/master'
* Merge branch 'rm-controls:master' into master
* Merge branch 'rm-controls:master' into master
* Update package.xml
* Fix pre-commit error
* Add MadgwickController configuration
* Add MadgwickController
* Add ImuFilterBase,overwrite ComplementaryController
* Merge branch 'master' of https://github.com/YuuinIH/rm_controllers
* Merge branch 'gimbal/opti_or_simplify' into chassis/balance_imu_interface
* Run pre-commit
* Update the config of imu_filter_controllers, load only one controller on launch instead of spawn controllers
* Publish imu angular velocity, linear acceleration, temperature and camer trigger time on three topics.
* Fix dt bug of ComplementaryController
* Put filtered quaternion into imu_extra_handle.
* Merge remote-tracking branch 'origin/imu_filter_controllers' into imu_filter_controllers
* Fix stupid bug on ComplementaryController
* Delete unnecessary
* Complete complementary controller.
* Delete imu filter.
* Correct format.
* Merge remote-tracking branch 'origin/chassis/fix_filter' into chassis/fix_filter
  # Conflicts:
  #	rm_chassis_controllers/src/chassis_base.cpp
  #	rm_chassis_controllers/src/swerve.cpp
* Modify code logic
* Finish version1 update which come true the publisher() function
* Add rm_common and hardware_interface dependence
* Fix class and namespace name error
* Add imu_filter_controllers
* Contributors: BruceLannn, QiayuanLiao, YuuinIH, kbxkgxjg, qiayuan, ye-luo-xi-tui, yezi
