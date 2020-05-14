aist_depth_filter: ROS node for filtering depth images
==================================================

This package provides a ROS node applying a series of filters to the input depth images, a python client class for utilizing functions of the node and some launch files.

# aist_depth_filter server

## Available filters

The node applies the filtering process to the input depth images in the following order;

### *Background subtraction* 

Compute differences in depth values between the input and the background depth images, then zero the depths at pixels with differences below the threshold. The backgournd depth image is saved in advance via the service `~/saveBG`. The threshold value is specified by the parameter `~thresh_bg`. The background subtraction is performed in advance of all the other filtering process, that is, the background image is saved as is and compared with the original incoming images withtout cropping.

### *Region of Interest(ROI)*

Sepcify a rectangular region of interest(ROI) of the input image and perform the subsequent filtering process only within this region. The ROI is determined by the upper-left corner (`~left`, `~top`) and lower-right corner (`~right`-1, `~bottom`-1) with four parameters `~top`, `~bottom`, `~left` and `~right`.

### *Z-clipping*

Depth values are zeroed if the original value is below the minimum or above the maximum distance values specified by the parameters `~near` and `~far` respectively.

### *Computing surface normals*

Compute surface normals from depth images.This is done by fitting a plane to the 3D point coodinates within a square window centered at the pixel of interest. The size of the window is 2*r*+1 where *r* is the radius specified by the parameter `~window_radius`.

### *Depth scaling*

Multiply a constant value specified by the parameter `~scale` to the input depths. This function is intended to be used for transforming the unit of distance or compensating errors in the depth scale of the 3D camera.

## ROS services

### `~/saveBG`

Save the original depth image with no filteres applied to `~/.ros/.tif`. This file is used as the backgroud in processing the subsequent input images.

### `~/capture`

First, capture the filtered intensity/color, depth and normal images. Then, create a PLY file from them and save it to `~/.ros/scene.ply`. Finally, its file path as well as the frame id of the camera is published to the topic `~/file_info`.

## ROS parameters

The following parameters can be specified not only in the command line at staring the node but also via `rqt_reconfigure` of the `dynamic_reconfigure` package in a dynamic and interactive manner;

- **~thresh_bg** -- Threshold value for background subtraction, thatis, if the difference between depth values in the subscribed and the backgroud images is below the threshold, the output depth will be zeroed at the corresponding pixel in the published depth image. If zero, no background subtraction is perforemd.
- **~top,~bottom,~left,~right** -- Specify ROI with the top-left corner at (left, top) and the bottom-right corner at (right-1, bottom-1)
- **~near,~far** -- Specify minimum and maximum depth values in z-clipping.
- **~subscribe_normal** -- If true, subscribe images of surface normals
- **~window_radius** -- Specify radius *r* of the square window for computing surface normals from depths when `~subscribe_normal` is false. If zero, no surface normals are computed.
- **~scale** -- Scaling factor for depth topic

## Subscribed ROS topics

- **/camera_info** -- Camera parameters including a 3x3 calibration matrix and lens distortions
- **/image** -- Intensity/color images
- **/depth** -- Depth images represented in meters with 32bit floating values or in milimeters with 16bit integer values
- **/normal** -- Images with 3D vectors representing surface normals(optional)

## Published ROS topics

- **~/camera_info** -- Camera parameters with intrinsics and image sizes modified according to the current ROI
- **~/image** -- Intensity/color images cropped with ROI
- **~/depth** -- Filtered depth images cropped with ROI
- **~/normal** -- Images with surface normals cropped with ROI
- **~/colored_normal** -- RGB images encoding surface normals cropped with ROI
- **~/file_info** -- File path of PLY file saved via the service `~/savePly` and corresponding frame id of the input depth image


# aist_depth_filter client

