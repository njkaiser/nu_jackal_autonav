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

You'll need to execute `remote-jackal.sh` on your control computer to set your environment variables as well. Change `ROS_HOSTNAME` to the name of your computer (mine is MacbookPro).

### Executing Custom Scripts on Startup
Clearpath wrote a nice walkthrough [here][1] on how to augment Jackal's boot process - follow this guide to add your own supplementations. I've added quite a few things - see **Additions** section below.

### Additions
1. `/etc/ros/indigo/ros.d/velodyne_startup.launch`: launch file to start up the necessary drivers/scripts for ROS communication with the Velodyne

2. `/opt/ros/indigo/share/jackal_description/urdf/jackal.urdf.xacro`: custom modifications to the default URDF for our Jackal setup

3. `$(find jackal_control)/configs/robot_localization.yaml`: addition of a `laser_scan_matcher` output (as `odom1`) to the odometry EKF

4. Added [`twist_mux`][2] capabilities. This allows us to have multiple sources controlling Jackal at the same time, with fluid handoffs between them, and without sources fighting one another for control. An example: if Jackal was navigating from A to B autonomously and you wanted to interrupt his motion with the PS3 controller, prior to `twist_mux` he would have tried to satisfy both commands and oscillated between the two. With `twist_mux` running, the joystick has higher priority than navigation, and will take control completely and immediately. Use `twist_mux_topics.yaml` in the `params` directory to add in new `cmd_vel` sources and determine their place in the heirarchy. The relevant files changed on Jackal:
  - `$(find jackal_control)/launch/control.launch`: `cmd_vel` changed to `cmd_vel/mux`
  - `$(find jackal_control)/launch/teleop.launch`: `cmd_vel` changed to `cmd_vel/joy` and `cmd_vel/marker`
  - `cmd_vel_relay` node: `cmd_vel` changed to `cmd_vel/mux`


<!-- Links -->
[1]: https://www.clearpathrobotics.com/assets/guides/jackal/startup.html
[2]: http://wiki.ros.org/twist_mux?distro=indigo
