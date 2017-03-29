Autonomous Navigation Project
==============
### using a Clearpath Jackal UGV and the ROS Navigation Stack

#### *Nate Kaiser -- MS in Robotics Winter Quarter Project -- Northwestern University*

![][1]

## Overview
This package, along with its dependencies, provide everything necessary to navigate the Northwestern Robotics Program's [Clearpath Jackal UGV][2] autonomously using laser data from a [Velodyne VLP-16 LIDAR][3].

Specifically, this package uses:
- Either the default [global_planner][4] or [NavFN][5] for global path planning
- Either [gmapping][6] or [hector_slam][7] for mapping/SLAM
- [AMCL][8] for localizing within a given map
- A custom local planner is in the works but is not yet fully implemented

Parameters have been tuned specifically for Northwestern's robot/environment combination, so some tweaks may be necessary if this is ported to a different robot/environment combination.


## Installation
#### Prerequisites
To install all the dependencies, fire up a terminal and run:

```bash
sudo apt-get update
sudo apt-get install ros-indigo-velodyne ros-indigo-velodyne-description ros-indigo-velodyne-simulator ros-indigo-jackal-desktop ros-indigo-jackal-gazebo
```

Alternatively, I hope to clean everything up and have `rosdep install name-of-package` functioning properly in the near future. **Important Note:** you may need to install `ros-indigo-velodyne` from source, per instructions [here][14] and [here][15], although installing from apt-get seems to be fully functioning when I last checked.

This will need to be done on both the host computer and Jackal's on-board computer, but Jackal should already have `ros-indigo-jackal-desktop` installed, and has no need for `ros-indigo-jackal-gazebo`, since Gazebo simulations will only be run on the host computer (though accidentally installing it won't hurt anything).

Note: this assumes you already have a ***full*** ROS Indigo installation (`ros-indigo-desktop-full`) running on Ubuntu 14.04. If not, you may have to install other dependencies that come with the full version (PCL, OpenCV, etc). Also, this package *should* work on a different ROS version or Linux version/distro, but no testing has been done to verify.

#### This Package
To install this package:
1. Clone [this repo][9] into a workspace and build\*
2. SSH into Jackal and repeat the process

This requires Jackal to be set up and connected to the internet. For help with this, Clearpath has provided an [excellent walkthrough][10].

\**I highly recommend using [catkin_tools][11], which has a modernized version of the catkin_make command and other useful command line tools for working with ROS/catkin packages*


## Instructions For Use
Navigate to your workspace and source your setup file: `source devel/setup.bash`. If you're running on the real robot, skip step 1, as it just launches the robot simulation. Choose only one of steps 4, 5, or 6, depending on what you want to do.

1. `roslaunch nu_jackal_autonav simulate.launch`

    This loads a URDF of Northwestern's Jackal configuration to the parameter server, starts up a Gazebo session with Clearpath's tutorial map, and spawns Jackal into it. By default, no GUI is shown, but can be shown by appending `gui:=true` to the end of the roslaunch command. See the beginning of the launch file for other helpful command line arguments.

2. `roslaunch nu_jackal_autonav pointcloud_filter.launch`

    This command eliminates the z drift between the `odom` and `base_link` frames and starts a nodelet manager to handle all of the pointcloud filtering. The outputs are a new `tf` frame named `odom_corrected` and a cascaded series of new pointcloud topics, the important one being `/velodyne_points/for_costmap`. The individual filters are:
    - a cropbox to eliminate any points further than 4 meters from Jackal in the x- and y-directions, and any points below 0.0 or above 0.4 meters in z
    - a statistical outlier filter (using `pcl/RadiusOutlierRemoval`)
    - a voxel grid downsampler
    - a custom filter which eliminates any points too close to the robot or within 1.5&deg; of the ground

3. `roslaunch nu_jackal_autonav laser_scan_operations.launch`

    I added some functionality and broke this node out as a separate entity instead of including it in multiple launch files. It produces an emulated `LaserScan` topic, runs an instance of `laser_scan_matcher`, and converts the output into an `odometry` topic for use by the the main `robot_localization` running on Jackal, which should help improve the odometry estimates.

4. `roslaunch nu_jackal_autonav odom_navigation.launch`

    This node is similar to the file in the `jackal_navigation` package with the same name. This puts Jackal in pure odometric navigation mode. Under the hood it just runs `move_base` and loads the correct parameters from yaml files stored in the `params` directory. All you have to do is send Jackal goal poses using RViz and watch as it navigates there autonomously.

5. `roslaunch nu_jackal_autonav gmapping.launch`

    Again, this is similar to its `jackal_navigation` counterpart. It starts `gmapping` and `move_base` with the correct parameters. Run this to make a map of whatever environment(s) Jackal will be navigating in. Drive around using the joystick controller until you're satisfied with the costmap (viewable in RViz). Only 1 map is needed for each environment. Also, please note you'll need to save the map once you're satisfied with it by running `rosrun map_server map_saver -f map-name-goes-here` in a separate terminal.

6. `roslaunch nu_jackal_autonav amcl.launch`

    This is the heart of Jackal's navigational capabilities. When working correctly, this node estimates the robot's pose in the map. The launch file starts [AMCL][8] and `move_base`. You'll need to modify the `map_file` parameter to point to the map you created in the last step (stored in `maps` directory).

7. `roslaunch nu_jackal_autonav view_rviz.launch`

    This runs one of two RViz sessions. If you're running `gmapping_demo` or `amcl_demo`, you'll need to append `config:=map` to the launch command. With this running, you can:
    - send a pose estimate to `AMCL`
    - send a navigation goal to `move_base`
    - control the robot using convenient and intuitive sliders
    - view the full Velodyne pointcloud
    - view the pointcloud used for determining costmap values
    - view the particle array of the particle filter
    - view the local and global costmaps
    - view the camera feed
    - view the local and global paths
    - view the emulated laser scan (used by `gmapping`/`AMCL`)


## Future Work
Currently, I'm working on a custom local planner plugin to replace the default [dwa_local_planner][13], since it's not ideal for this specific robot setup. I plan to release as a separate package when it's complete.

I'm also working on an RViz plugin for canceling navigation goals, but haven't had much time to work on this yet.


## Other Considerations
- you might need a decent, dedicated router for fast and uninterrupted data transfer
- if the laser location is changed, it will need to be updated in the URDF
- all parameters were set for vinyl lab floors, which are somewhat slick - different environments may require retuning
- nearby obstacles were always close proximity when developing, so parameters may need adjusting if this is used in a more open area
- this was developed in a highly dynamic environment, which may also require retuning some parameters as well
- most of the modifiable parameters are stored in yaml or launch files in the `params` or `launch` directories, respectively
- if navigation is acting funny or not working at all, check that `base_local_planner` isn't set to `custom_planner/CustomPlanner` in [launch/include/move_base.launch][12], since I'm not finished developing it yet
- OpenCV 2.4 was used, I have not checked compatibility with OpenCV 3
- PCL 1.7 was used, I have not checked compatibility with newer versions


<!-- File Locations -->
[1]: https://github.com/njkaiser/nu_jackal_autonav/blob/master/media/navigating_laser_only.gif
[2]: https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/
[3]: http://velodynelidar.com/vlp-16.html
[4]: http://wiki.ros.org/global_planner?distro=indigo
[5]: http://wiki.ros.org/navfn?distro=indigo
[6]: http://wiki.ros.org/gmapping?distro=indigo
[7]: http://wiki.ros.org/hector_slam?distro=indigo
[8]: http://wiki.ros.org/amcl?distro=indigo
[9]: https://github.com/njkaiser/nu_jackal_autonav
[10]: https://www.clearpathrobotics.com/assets/guides/jackal/network.html
[11]: https://catkin-tools.readthedocs.io/en/latest/
[12]: https://github.com/njkaiser/nu_jackal_autonav/blob/master/launch/include/move_base.launch#L26
[13]: http://wiki.ros.org/dwa_local_planner
