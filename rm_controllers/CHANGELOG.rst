^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
