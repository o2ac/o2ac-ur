aist_handeye_calibration: ROS package for estimating camera poses w.r.t. robots
==================================================

This package provides a set of software for estimating a relative transformation between a depth camera and a robotic arm. There are three main functions:

- **Calibrating cameras** -- Estimate a relative transformation between the camera and the robot by showing an AR marker to the camera while moving the robot to several known positions.
- **Checking calibration results** -- Validate the estimated parameters by showing a marker to the camera and then commanding the robot to move the tooltip to the center of the marker.
- **Publishing calibration results** -- Broadcast the transformation between the camera and the robot as a tf message so that vision programs can make use of it.

The calibration is possible in the following two situations:

- **eye_on_hand** -- Camera is mouted on the end-effector of the robot.
# 

1: Launch simulator
```bash
$ roslaunch o2as_gazebo o2as_gazebo.launch
```

2: Do following for `$CAMERA_NAME={a_bot_inside_camera, a_bot_outside_camera, b_bot_inside_camera, b_bot_outside_camera}`

2-1: Launch robots, cameras and the calibrator
```bash
$ roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch camera_name:=$CAMERA_NAME [sim:=true]
```
Gazebo is launched if sim:=true is specified while real robots are brought up otherwise. After robots appear in the screen, open the RViz config file `aist_handeye_calibration.rviz`. Then the marker detector and the hand-eye calibrator start.

2-2: Run calibration software in a different terminal
```bash
$ roslaunch aist_handeye_calibration run_calibration.launch camera_name:=$CAMERA_NAME
```

The `marker_id` in the yaml file may need to be changed from `32` to `26` for the OSX environment. The estimated parameters are stored in `~/.ros/$CAMERA_NAME.yaml`.

2-3: To use the calibrated camera frames, they need to be published from the calibration files:
```bash
$ roslaunch aist_handeye_calibration publish.launch camera_name:=$CAMERA_NAME
```

3: Calibration with real camera and robot

3-1: You need not launch gazebo.

3-2: Launch services for calibration

You need to specify `config:=real` when calibrating a real camera with a real robot. In addtion, you may need to specify marker ID according to the marker used in real situations. Since the marker ID attached to the top of workspace is 26 in OSX environment, you have to specify `marker_id:="26"` when calibrating `a_bot_camera` with `a_bot`:
```
$ roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch camera_name:=$CAMERA_NAME robot_name=$ROBOT_NAME config:=real [marker_id:="26"]
```
roslaunch aist_handeye_calibration o2ac_handeye_calibration.launch camera_name:="b_bot_outside_camera" robot_name="b_bot" config:=real marker_id:="26"

3-3: The estimation results can be visualized and the estimated parameters can be obtained with the same procedure as 2-3.
