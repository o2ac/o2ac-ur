# Run handeye calibration in simulator and real environment

1: Launch simulator
```bash
$ roslaunch o2as_gazebo o2as_gazebo.launch
```

2: Do following for `$CAMERA_NAME={a_bot_camera, a_phoxi_m_camera}` and `$ROBOT_NAME={a_bot, b_bot and c_bot}`

2-1: Launch services for calibration
```bash
$ roslaunch aist_handeye_calibration calibrate.launch camera_name:=$CAMERA_NAME robot_name=$ROBOT_NAME
```
After robots appear in the screen, open the RViz config file `aist_handeye_calibration.rviz`. Then the marker detector and the hand-eye calibrator start.

2-2: Run calibration software in a different terminal
```bash
$ rosrun aist_handeye_calibration run_handeye_calibration.py $CAMERA_NAME $ROBOT_NAME notrigger
```

2-3: Publish the estimated camera frames, loaded from the calibration files:
```bash
$ roslaunch aist_handeye_calibration publish.launch camera_name:=$CAMERA_NAME robot_name=$ROBOT_NAME
```
Estimated parameters are stored in `~/.ros/easy_handeye/aist_handeye_calibration_$ROBOT_NAME_eye_on[base|hand].yaml`.

3: Calibration with real camera and robot

3-1: You need not launch gazebo.

3-2: Launch services for calibration

You need to specify `config:=real` when calibrating a real camera with a real robot. In addtion, you may need to specify marker ID according to the marker used in real situations. Since the marker ID attached to the top of workspace is 26 in OSX environment, you have to specify `marker_id:="26"` when calibrating `a_bot_camera` with `a_bot`:
```
$ roslaunch aist_handeye_calibration o2as_calibrate.launch camera_name:=$CAMERA_NAME robot_name=$ROBOT_NAME config:=real [marker_id:="26"]
```

3-3: The estimation results can be visualized and the estimated parameters can be obtained with the same procedure as 2-3.
