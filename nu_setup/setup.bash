# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######

export JACKAL_PS3=1
export JACKAL_FLEA3=1
export JACKAL_FLEA3_MOUNT=front
export JACKAL_FLEA3_TILT="0.0"
export JACKAL_FLEA3_FRAME_RATE=30
# export ROS_HOSTNAME=CPR-J100-0076.local
export ROS_IP=192.168.0.100

# Pass through to the main ROS workspace of the system.
#source /opt/ros/indigo/setup.bash
source /home/administrator/catkin_ws/devel/setup.bash
