#!/bin/bash
ls
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-bloom fakeroot dh-make devscripts
pip install shyaml
package_version=`curl -sL https://github.com/ros/rosdistro/raw/master/noetic/distribution.yaml | shyaml get-value repositories.rm_control.release.version`
time_stamp=`date +%Y%m%d.%H%M%S`
rosdep update
for file in imu_filter_controllers rm_calibration_controllers rm_chassis_controllers rm_controllers rm_gimbal_controllers rm_orientation_controller rm_shooter_controllers robot_state_controller
do
  if test -d $file
  then
    echo "Trying to package $file"
    ls
    cd $file
    bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
    debchange -v $package_version.$time_stamp -p -D -u -m 'Append timestamp when binarydeb was built.'
    fakeroot debian/rules binary
    cd ..
  fi
done
ls
