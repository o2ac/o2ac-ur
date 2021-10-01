aist_handeye_calibration: a ROS package for estimating camera poses w.r.t. robots or the world
==================================================

## Overview
This package provides a set of software for estimating a relative transformation between a depth camera and a robotic arm. There are three functions:

- **Calibrating cameras** -- Estimate a relative transformation between the camera and the robot by showing an AR marker to the camera while moving the robot to several known keyposes.
- **Checking calibration results** -- Validate the estimated transformation by showing a marker to the camera and then commanding the robot to move its tooltip to the center of the marker.
- **Publishing calibration results** -- Broadcast the estimated transformation between the camera and the robot as a tf message so that vision programs can make use of it.

The calibration is possible under the following two cases:

- **eye_on_hand** -- Camera is mounted on the end-effector of the robot.
- **eye_on_base** -- Camera is fixed to the stationary environment.

## Step 0: Preparing an AR marker

A square AR marker, [ArUco](http://www.uco.es/investiga/grupos/ava/node/26), is required for calibration. Default marker used in this package is the one of ID=26, size=0.07 and margin=0.005 which can be found in `aist_handeye_calibration/markers/aruco-26-70-5.pdf`. Please print and paste it to a square plate of 80mm x 80mm size. Then, you should fix the marker plate to the environment(`eye_on_hand`) or to the end-effector(`eye_on_base`).

If you want to use a marker with different ID or size, please visit [marker generator site](https://chev.me/arucogen). You should select `Original ArUco` dictionary and specify marker ID as well as its size you want.
The configuration parameters also should be modified in accordance with the marker properties. Please see the section [Parameters for configuring calibration](#Parameters) for details.

## Step 1: Calibrating cameras

First, you bring up robots, cameras and the calibrator, i.e. *calibration server*, with the following command:
```bash
$ roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch [sim:=true] camera_name:=<camera name>
```
where `<camera name>` is the name of the camera to be calibrated, e.g. `camera_name:=a_bot_inside_camera` . The calibration process is simulated with gazebo if `sim:=true` is specified, or performed with real robots and cameras otherwise.

Next, the *calibration client* should be launched in another terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration run_calibration.launch camera_name:=<camera name>
```
Here, `<camera name>` must be same as the one specified for the server.

After the comes up, please simply hit **return** key to start the calibration process. The robot moves to several key poses at each of which the client takes an image of the marker and estimates its pose. After visiting all the key poses, a 3D transformation from the camera to the robot is computed and saved in `o2ac_scene_description/config/camera_calibration/<camera_name>.yaml`. The estimated transformation is also displayed in the terminal which has launched the client together with the residual errors.

You can terminate the client by hitting **q** and **return**.

## Step 2: Checking calibration results

You can validate the calibration result by checking whether the robot can move its tooltip to the target position specified by the marker placed at an arbitrary position.

First, you should bring up robots, cameras and the *calibration publisher* with the following command:
```bash
$ roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch [sim:=true] camera_name:=<camera name> check:=true
```
where `<camera name>` is the name of the calibrated camera.

Next, the *checking client* should be launched in another terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration check_calibration.launch camera_name:=<camera name>
```
Here, `<camera name>` must be same as the one specified for the publisher.

After the client comes up, please simply hit **i** and **return** keys to move the robot to the initial position. Then place the marker at an arbitrary position with an arbitrary orientation within the camera's field of view and hit **return**. The robot will go to the approach position 50mm above the marker and descend to the center along the normal direction. You can repeat this process varying the marker positions and orientations.

You can terminate the client by hitting **q** and **return**.

## Step 3: Publishing calibration results

In order to make the calibration results available to the vision programs, they need to be published to `tf` as a transformation. This can be done with the *calibration publisher* which can be launched by:
```
$ roslaunch aist_handeye_calibration publish_calibration.launch camera_name:=<camera name>
```
where `<camera name>` is the name of the calibrated camera. The ID of the published camera frame is prefixed with `calibrated_<camera_name>`. For example, the ID of the frame associated with color images of the `a_bot_inside_camera` will become `calibrated_a_bot_inside_camera_color_optical_frame`.

## Parameters for configuring calibration

The calibration process can be customized by adjusting the parameters listed below. They are stored in `aist_handeye_calibration/config/o2ac/<camera_name>.yaml` where `<camera_name>` is the name of the camera to be calibrated.
- **camera_type** -- If the camera continuously streams the depth and intensity/color images, this value should be *DepthCamera*. Otherwise, a special camera type should be specified according to the API for triggering image frames. Currently, only *PhoXiCamera* is supported.
- **camera_name** -- Name of the camera, e.g. *a_bot_inside_camera*
- **robot_name** -- Name of the robot used for calibration, e.g. *a_bot*. In the **eye_on_hand** configurations, the name of the robot on which the camera is mounted should be given. In the **eye_on_base** configurations, name of the robot to which the marker attached should be specified.
- **eye_on_hand** -- *true* for the **eye_on_hand** configurations while *false* for **eye_on_base** configurations.
- **robot_base_frame** -- ID of the frame with respect to which the cartesian robot poses are specified, e.g. *workspace_center*.
- **robot_effector_frame** -- ID of the frame attached to the end-effector of the robot,e.g. *a_bot_ee_link*. When calibrating a camera, robot motions are given as poses of this frame with respect to the *robot_base_frame*.
- **camera_frame** -- ID of the frame associated with the depth and intensity/color images, e.g. *calibrated_a_bot_inside_camera_color_optical_frame*. This value must be identical to that specified when launching tha camera. In addition, depth images must be aligned to intensity/color images so that they have a common projection center.
- **marker_id** -- ID of the marker
- **marker_size** -- Side length of the square marker (in meters). Margins are not included.
- **planarity_tolerance** -- Thresholding value for filtering out outliers when fitting a plane to the 3D points within a detected marker region in the depth image (in meters).
- **initpose** -- Initial robot pose when checking calibration results. The pose is given by a 6-dimensional array with x/y/z coordinates (in meters) and roll/pitch/yaw angles (in degrees) with respect to the *robot_base_frame*.
- **keyposes** -- An array of robot poses where marker images are captured when calibrating the camera. Each pose is given by a 6-dimensional array with x/y/z coordinates (in meters) and roll/pitch/yaw angles (in degrees) with respect to the *robot_base_frame*.

## Calibration algorithm

The calibrator implements the following two algorithms for estimating a camera pose from a set of transformation pairs composed of the one from the end-effector to the robot base and the one from the marker to the camera.
- **Classical method** -- This algorithm first optimizes camera orientation and then optimizes camera rotation while the orientation is fixed to the value estimated before.
- **Dual quaternion method** -- This algorithm optimizes both camera position and orientation simultaneously, which gives more accurate results compared with the classical method. See
```
K.Daniilidis and E. Bayro-Corrochano, The dual quaternion approach to hand-eye calibration, Int. J. Robotics Research, 18: 286-298, 1999
```
If `use_dual_quaternion:=false` is specified when launching the calibrator, the former algorithm is used. The latter is adopted otherwise.