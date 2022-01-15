^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rm_gimbal_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
