aist_localization: ROS node for matching 3D CAD models with depth images
==================================================

This package provides a ROS node which matches 3D CAD models to the input depth images, a python client class for utilizing functions of the node and some launch files.

## aist_localization server

The node is a ROS wrapper for the [Photoneo Localization SDK](https://www.photoneo.com/products/photoneo-3d-localization-c-sdk/). Since the SDK only accepts outputs from Photoneo PhoXi cameras or point cloud files stored in Stanford PLY format as input 3D data, the node is designed to be used in conjunction with `aist_depth_fitler` which creates point clouds from input depth images and saves in PLY files.

## 3D CAD models

The 3D CAD models need to be given in Stanford PLY format. This is due to the requirement incurred by the [Photoneo Localization SDK](https://www.photoneo.com/products/photoneo-3d-localization-c-sdk/). The path to the directory containing models is given by the parameter `~ply_dir` whose default value is `o2ac_parts_description/meshes`.

## Subscribed ROS topics

The localization server subscribes **/file_info** which contains a path of PLY file saved by the `aist_depth_filter` as well as an equation of the dominant plane. The normal and distance of the dominant plane are also included if detected.


# aist_localization client

The package also provides a simple client class `aist_localization_filter.LocalizationClient` for easing development of applications with python. Please refer to `aist_localization/src/aist_localization/__init__.py`.


# Testing

The results of localization from live camera images can be displayed in RViz with the following command
```bash
$ roslaunch aist_localization phoxi.launch
```
for PhoXi cameras and
```bash
$ roslaunch aist_localization realsense.launch
```
for RealSense cameras. The localization parameters can be adjusted interactively through `rqt_reconfigure`.