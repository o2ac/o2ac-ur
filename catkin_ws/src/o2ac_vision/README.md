# Introduction

This package contains nodes that execute and advertise vision actions, e.g.:

- Part recognition
- Tooltip alignment
- Visual servoing
- Orientation checks

All vision skills should be action-based, so that calculations are allowed to fail and time out.

For this, the Python nodes should advertise a number of actions, which are defined in o2ac_msgs.

## Part recognition
Part recognition node consist of two compornents. One is the object detection, the other is pose estimation.

### Object detection
M2det

### Pose estimatrion
This component feeds the output of object detection, a list of bounding box and object class id, and estimates acculate pose (x,y,theta) of targets in image coordinate system.

You can try this componet using this component:
```
$ python pose_estimation_test.py --cimg [filename] --timg [filename]
```
Options --cimg and --timg take file name of the input image and template image, respectively.

Test image set can be download from [here](https://drive.google.com/drive/u/0/folders/1JIA7FTcoSxNIfv80T5eiqRywfnHOizXK).
