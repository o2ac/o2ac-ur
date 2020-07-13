^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosparam_shortcuts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2019-09-24)
------------------
* Switch run_depend to eigen_conversions. (`#12 <https://github.com/PickNikRobotics/rosparam_shortcuts/issues/12>`_)
  Eigen is header-only (so no run_depend needed), but eigen_conversions
  has a library that needs to be pulled in at runtime.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

0.3.2 (2019-09-18)
------------------
* Update to new ros_buildfarm workspace directory
* Add get() for geometry_msgs::Pose
* Contributors: Henning Kayser

0.3.1 (2019-04-12)
------------------
* removing deprecated functions because catkin can't tell the difference between affine3d and isometry3d (`#9 <https://github.com/picknikrobotics/rosparam_shortcuts/issues/9>`_)
* Travis badge fixup (`#8 <https://github.com/picknikrobotics/rosparam_shortcuts/issues/8>`_)
  * fixing travis and build tags
  * updating install instructions
  * adding melodic ci
* Contributors: Mike Lautman

0.3.0 (2019-04-10)
------------------
* Deprecate Affine3d transforms and support Isometry3d (`#7 <https://github.com/picknikrobotics/rosparam_shortcuts/issues/7>`_)
* Merge pull request `#4 <https://github.com/picknikrobotics/rosparam_shortcuts/issues/4>`_ from PickNikRobotics/kinetic-devel-eigen-additions
  adding support for loading trajectories and loading quaternions
* removing EigenSTL
* adding support for loading trajectories and loading quaternions
* Merge pull request `#3 <https://github.com/picknikrobotics/rosparam_shortcuts/issues/3>`_ from gavanderhoorn/patch-1
  Fix minor typo in API doc link.
* Fix minor typo in API doc link.
* Contributors: Dave Coleman, G.A. vd. Hoorn, Henning Kayser, mike

0.2.1 (2016-09-28)
------------------
* Fix Eigen3 include
* Fix C++11 compiling method
* Update Travis for Kinetic
* Updated README
* Contributors: Dave Coleman

0.2.0 (2016-06-29)
------------------
* Upgrade to Eigen3 per ROS Kinetic requirements
* Converted to C++11 for ROS Kinetic
* Removed deprecated functions
* Improve documentation
* fix typo
* Added loading a vector to example
* Contributors: Dave Coleman, Sammy Pfeiffer

0.1.1 (2015-12-18)
------------------
* Fixed CMake build
* Ran roslint
* Ran catkin lint
* Updated example
* Contributors: Dave Coleman

0.1.0 (2015-12-17)
------------------
* Deprecated long named functions in favor of short named functions
* Removed redundant unsigned int function
* Contributors: Dave Coleman

0.0.7 (2015-12-13)
------------------
* Added shortcut version of functions
* Fix install
* Contributors: Dave Coleman

0.0.6 (2015-12-10)
------------------
* Added example code
* Contributors: Dave Coleman

0.0.5 (2015-12-09)
------------------
* Attempting to fix Eigen
* Fix missing dependency
* Contributors: Dave Coleman

0.0.4 (2015-12-07)
------------------
* Attempt to fix Eigen include dir
* Contributors: Dave Coleman

0.0.3 (2015-12-05)
------------------
* Added travis support
* catkin lint cleanup
* Switched travis to jade branch
* Contributors: Dave Coleman

0.0.2 (2015-12-03)
------------------
* Initial release
* Contributors: Dave Coleman
