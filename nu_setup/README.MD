Instructions & Notes for NU MSR Students
==============
### Overview
For any future Northwestern MSR students who want to use Jackal, continue reading for a bit of background on how to make lower-level changes to the robot. If you have any questions, please contact Jarvis or me.

### Modified Jackal Configuration Files
When Jackal was freshly configured, 3 files were modified to allow bi-directional wireless networking:

1. /usr/sbin/ros-start
2. /etc/init/ros.conf
3. /etc/ros/setup.bash

These changes basically set the correct `ROS_IP` environment variable correctly. The files are included in the repo with this README, should something happen to Jackal's computer. Any modifications we made have been commented in each of the files. **Note:** these should *NOT* need to changed again unless the on-board computer or OS is modified.

### Executing Custom Scripts on Startup
Clearpath wrote a nice walkthrough [here][1] on how to augment Jackal's boot process - follow this guide to add your own supplementations. Currently, I've added the following:

1. Custom modifications to the default URDF for our Jackal setup (/opt/ros/indigo/share/jackal_description/urdf/jackal.urdf.xacro)
2. A launch file to start up the necessary drivers/scripts for ROS communication with the Velodyne (/etc/ros/indigo/ros.d/velodyne_startup.launch )

<!-- Links -->
[1]: https://www.clearpathrobotics.com/assets/guides/jackal/startup.html
