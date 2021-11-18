#!/bin/bash
ls
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-bloom fakeroot dh-make
rosdep update
for file in imu_filter_controllers rm_calibration_controllers rm_chassis_controllers rm_controllers rm_gimbal_controllers rm_orientation_controller rm_shooter_controllers robot_state_controller
do
  if test -d $file
  then
    echo "Trying to package $file"
    ls
    cd $file
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    fakeroot debian/rules binary
    cd ..
  fi
done
ls
