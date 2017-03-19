Winter Quarter Project: Autonomous Navigation
==============
### Using a Clearpath Jackal UGV and the ROS Navigation Stack

#### *Nate Kaiser - Northwestern MS in Robotics Winter Quarter Project*

![][1]

## Overview
This package, along with its dependencies, provide everything necessary to navigate the Northwestern Robotics Program's [Clearpath Jackal UGV][2] autonomously using laser data from a [Velodyne VLP-16 LIDAR][3].

Specifically, this package uses:
- Either the default [global_planner][4] or [NavFN][5] for global path planning
- Either [gmapping][6] or [hector_slam][7] for mapping/SLAM
- [AMCL][8] for localizing within a given map
- A custom local planner is in the works but is not yet fully implemented

Parameters have been tuned specifically for Northwestern's robot/environment combination, so some tweaks may be necessary if this is ported to a different robot or environment.


## Installation
#### Prerequisites
To install all the dependencies, fire up a terminal and run:

<!-- `sudo apt-get update`

`sudo apt-get install ros-indigo-velodyne ros-indigo-velodyne-description ros-indigo-velodyne-simulator ros-indigo-jackal-desktop ros-indigo-jackal-gazebo` -->

```bash
sudo apt-get update
sudo apt-get install ros-indigo-velodyne ros-indigo-velodyne-description ros-indigo-velodyne-simulator ros-indigo-jackal-desktop ros-indigo-jackal-gazebo
```

Alternatively, I hope to add `rosdep install name-of-package` functionality in the near future.

This will need to be done on both the host computer and Jackal's computer, but Jackal should already have `ros-indigo-jackal-desktop` installed, and has no need for `ros-indigo-jackal-gazebo`, since Gazebo simulations will only be run on the host computer (though installing it won't hurt anything).

Note: this assumes you already have a ***full*** ROS Indigo installation (`ros-indigo-desktop-full`) running on Ubuntu 14.04. If not, you may have to install other dependencies that come with the full version (PCL, OpenCV, etc). Also, this package *should* work on a different ROS version or Linux version/distro, but no testing has been done to verify.

#### This Package
To install this package:
1. Clone [this repo][9] into a workspace and build\*
2. SSH into Jackal and repeat the process

This requires Jackal to be set up and connected to the internet. For help with this, Clearpath has provided an [excellent walkthrough][10].

\**I highly recommend trying [catkin_tools][11], which has a modernized version of the catkin_make command and other useful command line tools for working with ROS/catkin packages*


## Instructions For Use
Navigate to your workspace and source your setup file: `source devel/setup.bash`. If you're running on the real robot, skip steps 2-3, as they just start the simulated environment.

1. `roslaunch winter_project simulate.launch`

    This loads a URDF of Northwestern's Jackal configuration to the parameter server, starts up a Gazebo session with Clearpath's tutorial map, and spawns Jackal into it. By default, no GUI is shown, but can be added by appending `gui:=true` to the end of the roslaunch command

2. `roslaunch winter_project ground_plane.launch`

    This command eliminates the z drift between the `odom` and `base_link` frames and starts a nodelet manager to handle all of the pointcloud filtering. The outputs are a new `tf` frame, `odom_corrected`, and a cascaded series of new pointcloud topics, the last one (and only one we care about) being `/velodyne_points/for_costmap` The filters are:
    - a cropbox to eliminate any points further than 4 meters from Jackal in the x- and y-directions
    - a statistical outliers filter (using `pcl/RadiusOutlierRemoval`)
    - a voxel grid downsampler
    - a custom filter which eliminates any points too close to the robot or within 1.5&deg; of the ground



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
[4]: http://wiki.ros.org/global_planner?distro=indigo
[5]: http://wiki.ros.org/navfn?distro=indigo
[6]: http://wiki.ros.org/gmapping?distro=indigo
[7]: http://wiki.ros.org/hector_slam?distro=indigo
[8]: http://wiki.ros.org/amcl?distro=indigo
[9]: https://github.com/njkaiser/Winter_Project
[10]: https://www.clearpathrobotics.com/assets/guides/jackal/network.html
[11]: https://catkin-tools.readthedocs.io/en/latest/
[12]:
[13]:
