# ROS Param Shortcuts

Quickly load variables from rosparam with good command line error checking.

This package enforces the philosophy that there should be no default parameters - everything must be defined by the user in yaml files (or launch files or where ever) otherwise your program should not run. This helps debug why something isn't working correctly - it will tell you exactly what rosparameters are missing.

Features:
 - Outputs all loaded data into consule using ROS_DEBUG, so you won't see it unless you turn it on
 - Namespaces all output within the ``parent_name``
 - Great for having each class have its own parameter namespace
 - Helpful error messages if parameter is missing, explaining where it expects to find it
 - Removes lots of repetitious code
 - Supports datatypes that rosparam does not by default, such as std::size_t, ros::Duration, Eigen::Isometry3d, Eigen::Affine3d (deprecated)
 - Supports loading std::vectors easily, and debugging that data
 - Supports loading an entire list of bool parameters

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder

Status:

 * [![Build Status](https://api.travis-ci.org/PickNikRobotics/rosparam_shortcuts.svg?branch=melodic-devel)](https://travis-ci.org/PickNikRobotics/rosparam_shortcuts) Travis - Continuous Integration (Melodic)
 * [![Build Status](https://api.travis-ci.org/PickNikRobotics/rosparam_shortcuts.svg?branch=kinetic-devel)](https://travis-ci.org/PickNikRobotics/rosparam_shortcuts) Travis - Continuous Integration (Kinetic)
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__rosparam_shortcuts__ubuntu_bionic__source)](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__rosparam_shortcuts__ubuntu_bionic__source/) ROS Buildfarm - Bionic - Melodic Devel - Source Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rosparam_shortcuts__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__rosparam_shortcuts__ubuntu_bionic_amd64__binary/) ROS Buildfarm - AMD64 Bionic - Melodic Devel - Debian Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__rosparam_shortcuts__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__rosparam_shortcuts__ubuntu_xenial__source/) ROS Buildfarm - Xenial - Kinetic Devel - Source Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__rosparam_shortcuts__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__rosparam_shortcuts__ubuntu_xenial_amd64__binary/) ROS Buildfarm - AMD64 Xenial - Kinetic Devel - Debian Build
## Install

### Ubuntu Debian

`rosparam_shortcuts` is currently only released for ROS Kinetic and Melodic. For other ROS releases, you may need to build `rosparam_shortcuts` from source.

    sudo apt-get install ros-$ROS_DISTRO-rosparam-shortcuts

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

## Code API

See [Class Reference](http://docs.ros.org/kinetic/api/rosparam_shortcuts/html/)

## Usage / Demo

See the file ``src/example.cpp`` for example code. To run:

    roslaunch rosparam_shortcuts example.launch

Your yaml file would look something like the file ``config/example.yaml``:

```
example:
  control_rate: 100.0 # double
  param1: 20 # int
  param2: 30 # size_t
  param3: 1 # ros::Duration
  param4: [1, 1, 1, 3.14, 0, 0] # Eigen::Isometry3d - x, y, z, roll, pitch, yaw
  param5: [1.1, 2.2, 3.3, 4.4] # std::vector<double>
  param6: [2, 2, 2, 1, 0, 0, 0] # Eigen::Isometry3d - x, y, z, qw, qx, qy, qz
  param7: [1, 1, 1, 3.14, 0, 0] # geometry_msgs::Pose - x, y, z, roll, pitch, yaw
  param8: [2, 2, 2, 1, 0, 0, 0] # geometry_msgs::Pose - x, y, z, qw, qx, qy, qz
```

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
