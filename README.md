Winter Quarter Project: Autonomous Navigation
==============
### Using Clearpath Jackal UGV & the ROS Navigation Stack

#### *Nate Kaiser - Northwestern MS in Robotics Winter Quarter Project*

![][1]

## Overview
This package, along with its dependencies, provide everything necessary to navigate the Northwestern Robotics Program's [Clearpath Jackal UGV][2] autonomously using laser data from a [Velodyne VLP-16 LIDAR][3].

Specifically, this package uses:
- Either the default global_planner or navFN for global plan generation
- Either gmapping or hector_slam for SLAM/mapping
- AMCL for localizing within a given map
- A custom local planner is in the works but is not yet fully implemented

Parameters have been tuned specifically for Northwestern's robot/environment combination, so

WHAT DOES THIS PROJECT DO?


## Instructions
HOW DO THEY USE IT?


## Installation
HOW DO THEY GET/INSTALL IT on comp/then on Jackal?
- download my package and build
  - I recommend catkin build tools (link to apt-get command)
- install and set up communication w/ Jackal

-ubuntu 14.04, ros indigo (full install includes PCL/OpenCV)
rosdep install _
or sudo apt-get _


## Specifics
- xyz location of the laser
- configs set for floor type (slick for wheels)
- proximity to stuff when navigating = close, also highly dynamic
- most modifiable parameters stored in config files or launch files


## Future Work
- currently working on custom local planner, will release as separate package when finished


## Other Considerations?
- fast, empty router necessary?


<!-- File Locations -->
[1]: https://github.com/njkaiser/Winter_Project/blob/master/media/navigating_laser_only.gif
[2]: https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/
[3]: http://velodynelidar.com/vlp-16.html
