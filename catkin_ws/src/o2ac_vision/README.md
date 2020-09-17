# Introduction

This package contains nodes that execute and advertise vision actions, e.g.:

- Object detection
- Part pose estimation
- Belt pick-pose detection

All vision skills should be action-based, so that calculations are allowed to fail and time out.

For this, the Python nodes should advertise a number of actions, which are defined in o2ac_msgs.


## Part recognition
The part recognition node consists of two components. One is the object detection, the other is pose estimation.


### Object detection
Python scripts of Single Shot MultiBox Detector (SSD) are cloned from [ssd.pytorch](https://github.com/amdegroot/ssd.pytorch).
This module detects multiple objects in a tray. A list of bounding boxes, classes, and confidences are returned.


### Pose estimation
This component estimates accurate pose (x,y,theta) of **small targets** in image coordinate system. It feeds the output of the object detection module, a list of bounding box and object class id.

You can try this component using the following commands:
```
$ rosrun o2ac_vision pose_estimation_server.py
$ rosrun o2ac_vision pose_estimation_client.py --id [image id] --tdir [path]
```
Options:  
--id ... Index of the input image.  
--tdir ... Path to template info (template_info.json).  


### Belt detection
You can try this component using the following commands:
```
$ rosrun o2ac_vision belt_detection_server.py
$ rosrun o2ac_vision belt_detection_client.py
```


### Dataset
All data including the pre-trained model of SSD, templates, and image sets can be download from following link.
Please put `dataset.zip` in the directory "src/WRS_Dataset" and unzip it.
Make sure "Annotations", "Images", "data", "labels.txt", "realsense_intrinsic.json", and "ssd.pytorch" are in "src/WRS_Dataset".

[Download LINK](https://since1954-my.sharepoint.com/:f:/g/personal/z119104_since1954_onmicrosoft_com/EjnbKhpQsTRGnJWvP5ivM9sB3IzRr7gdRk0klG6oxHJyAQ?e=A3sxj1)


### Test
You can try this component using the following commands:
```
$ rosrun o2ac_vision ssd_server.py
$ rosrun o2ac_vision pose_estimation_server.py
$ rosrun o2ac_vision belt_detection_server.py
$ rosrun o2ac_vision server.py
$ rosrun o2ac_vision client.py --id [image id] --tdir [path]
```
