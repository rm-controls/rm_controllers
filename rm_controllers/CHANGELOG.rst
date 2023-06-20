^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#85 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/85>`_ from ye-luo-xi-tui/master
  0.1.7
* Contributors: ye-luo-xi-tui

0.1.7 (2022-09-10)
------------------
* Merge remote-tracking branch 'origin/master'
* Contributors: qiayuan

0.1.6 (2022-06-16)
------------------
* Merge pull request `#76 <https://github.com/rm-controls/rm_controllers/issues/76>`_ from Edwinlinks/tof-radar-controller
  Finished tof radar controller and delete tof sensor controller
* Change exec depend tof_sensor_controller to tof_radar_controller
* Merge remote-tracking branch 'origin/master'
* Contributors: Edwinlinks, QiayuanLiao, qiayuan

0.1.5 (2022-06-10)
------------------
* Merge remote-tracking branch 'origin/master'
* 0.1.4
* Merge remote-tracking branch 'origin/master'
* Merge pull request `#60 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/60>`_ from ye-luo-xi-tui/master
  Add subpackage to metapackage's package.xml
* Merge branch 'orientation_controller'
* Add all subpackages to metapackage.
* Merge branch 'master' into gimbal_track
* Merge pull request `#46 <https://github.com/ye-luo-xi-tui/rm_controllers/issues/46>`_ from ye-luo-xi-tui/master
  Deprecated imu_filter_controller
* Contributors: QiayuanLiao, YuuinIH, qiayuan, yezi

0.1.3 (2022-03-28)
------------------
* Deprecated imu_filter_controller(Since the update frequency of the control loop is not stable, some of
  the camera trigger signals of imu will be lost. We put the imu filter down to the hardware resource layer, so
  imu_extra_handle is breaking. )
* Contributors: qiayuan, ye-luo-xi-tui, yezi

0.1.2 (2022-01-08)
------------------

0.1.1 (2021-08-12)
------------------
* Set all version to the same
* Add license to robot_state_controller source files
* Add rm_controllers
* Contributors: qiayuan
