Winter Quarter Project: Autonomous Navigation
==============
### Using a Clearpath Jackal UGV and the ROS Navigation Stack

#### *Nate Kaiser -- MS in Robotics Winter Quarter Project -- Northwestern University*

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

Alternatively, I hope to have `rosdep install name-of-package` functioning properly in the near future.

This will need to be done on both the host computer and Jackal's on-board computer, but Jackal should already have `ros-indigo-jackal-desktop` installed, and has no need for `ros-indigo-jackal-gazebo`, since Gazebo simulations will only be run on the host computer (though accidentally installing it won't hurt anything).

Note: this assumes you already have a ***full*** ROS Indigo installation (`ros-indigo-desktop-full`) running on Ubuntu 14.04. If not, you may have to install other dependencies that come with the full version (PCL, OpenCV, etc). Also, this package *should* work on a different ROS version or Linux version/distro, but no testing has been done to verify.

#### This Package
To install this package:
1. Clone [this repo][9] into a workspace and build\*
2. SSH into Jackal and repeat the process

This requires Jackal to be set up and connected to the internet. For help with this, Clearpath has provided an [excellent walkthrough][10].

\**I highly recommend trying [catkin_tools][11], which has a modernized version of the catkin_make command and other useful command line tools for working with ROS/catkin packages*


## Instructions For Use
Navigate to your workspace and source your setup file: `source devel/setup.bash`. If you're running on the real robot, skip step 1, as it just launches the simulated environment. Choose only one of steps 3, 4, or 5, depending on what your

1. `roslaunch winter_project simulate.launch`

    This loads a URDF of Northwestern's Jackal configuration to the parameter server, starts up a Gazebo session with Clearpath's tutorial map, and spawns Jackal into it. By default, no GUI is shown, but can be added by appending `gui:=true` to the end of the roslaunch command. See the beginning of the launch file for other helpful command line arguments.

2. `roslaunch winter_project ground_plane.launch`

    This command eliminates the z drift between the `odom` and `base_link` frames and starts a nodelet manager to handle all of the pointcloud filtering. The outputs are a new `tf` frame named `odom_corrected` and a cascaded series of new pointcloud topics,the important one being `/velodyne_points/for_costmap`. The individual filters are:
    - a cropbox to eliminate any points further than 4 meters from Jackal in the x- and y-directions, and any points outside of below 0 or above 0.4 meters in z
    - a statistical outliers filter (using `pcl/RadiusOutlierRemoval`)
    - a voxel grid downsampler
    - a custom filter which eliminates any points too close to the robot or within 1.5&deg; of the ground

3. `roslaunch winter_project odom_navigation_demo.launch`

    This node is similar to the file in the `jackal_navigation` package with the same name. This puts Jackal in pure odometric navigation mode. Under the hood it just runs `move_base` and loads the correct parameters from yaml files stored in the `params` directory. All you have to do is send Jackal goal poses using RViz and watch as it navigates there autonomously.

4. `roslaunch winter_project gmapping_demo.launch`

    Again, this is similar to its `jackal_navigation` counterpart. It starts `gmapping` and a `pointcoud_to_laserscan` node, since `gmapping` needs a `LaserScan` topic to function. Run this to make a map of whatever environment(s) Jackal will be navigating in. Drive around using the joystick controller until you're satisfied with the costmap (viewable via RViz). Only 1 map is needed for each environment. Also, please note you'll need to save the map once you're satisfied with it by running `rosrun map_server map_saver -f map-name-goes-here` in a separate terminal.

5. `roslaunch winter_project amcl_demo.launch`

    This is the heart of Jackal's navigational capabilities. When working correctly, this node estimates the robot's pose in the map. The launch file starts [AMCL][8] and `pointcoud_to_laserscan` (to emulate a LaserScan topic needed for AMCL). You'll need to modify the `map_file` parameter to point to the map file created in the last step.

6. `roslaunch winter_project view_rviz.launch`

    This runs one of two RViz sessions. If you're running `gmapping_demo` or `amcl_demo`, you'll need to append `config:=amcl` to the launch command. With this running, you can:
    - send a pose estimate to `AMCL`
    - send a goal pose to `move_base`
    - control the robot using convenient and intuitive sliders
    - view the full Velodyne pointcloud
    - view the pointcloud used for determining costmap values
    - view the particle array of the particle filter
    - view the local and global costmaps
    - view the camera feed
    - view the local/global paths
    - view the emulated laser scan (used by `gmapping`/`AMCL`)


## Other Considerations
- you might need a decent, dedicated router for fast and uninterrupted data transfer
- if the laser location is changed, it will need to be updated in the URDF
- all parameters were set for vinyl lab floors, which is somewhat slick - different environments may require re-tuning
- nearby obstacle proximity was always close when developing/tuning, so parameters may need adjusting if this is used in a more open area
- the lab this was developed in is also a highly dynamic environment, so this may change some of the parameters as well
- most of these modifiable parameters are stored in yaml or launch files in the `params` or `launch` directories, respectively
- if navigation isn't working or acting funny, check that `base_local_planner` isn't set to `custom_planner/CustomPlanner`, since I'm not finished developing it yet (see [this file][12])

## Future Work
Currently, I'm working on a custom local planner plugin to replace the default [DWA][13], since it doesn't work that well for this specific robot setup. I plan to release it as separate package when it's complete.

I'm also working on an RViz plugin for canceling navigation goals, but haven't had much time to work on this one yet.


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
[12]: https://github.com/njkaiser/Winter_Project/blob/master/launch/include/move_base.launch#L26
[13]: http://wiki.ros.org/dwa_local_planner
