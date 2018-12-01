# cxq41_ps8_box_unloader

In order to run this package, you will need something called cwru_ariac_2018, link here: https://github.com/cwru-robotics/cwru_ariac_2018


What's more, you will need Dr. Newman's Learning ROS repository extra (You can delete everything related to baxter): https://github.com/wsnewman/learning_ros_external_pkgs_kinetic.git


## Example usage

There is a unified launch shell under LaunchShell, give it chmod 777 and ./launch.sh


This shell script will run the two launch files with a difference 30 seconds timing.


If this don't work, run the two launchfile in the launch folder, first main, then secondary.

## Running tests/demos
    
To run the node, rosrun cxq41_ps8_box_unloader unload_box.cpp