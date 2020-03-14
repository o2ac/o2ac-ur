aist_phoxi_camera: ROS driver for PhoXi 3D Scanner
==================================================

This package provides a ROS driver for controlling [PhoXi 3D
scanners](http://www.photoneo.com/phoxi-3d-scanner). The software is
inspired by [ROS drvier](https://github.com/photoneo/phoxi_camera) by
the manufacturer, i.e. [Photoneo co.](https://www.photoneo.com),
however, its structure is completely reorganized.

## Installation

Before compiling `aist_phoxi_camera`, you have to install
[PhoXiControl](http://www.photoneo.com/3d-scanning-software) which is
a GUI-based controller for PhoXi 3D scanners provided by `Photoneo
co.`.

Currently `PhoXiControl` is formally supported on Ubuntu 14 and Ubuntu
16 only. However it can be executed on Ubuntu 18 by installing some
additional packages. You can install `PhoXiControl` as well as these
extra packages by simply typing;
```bash
$ cd (your-catkin-workspace)/src/aist_phoxi_sensor/install-scripts
$ sudo ./install-photoneo.sh
```
Then append the following lines to your `~/.bashrc`
```bash
if [ -d /opt/PhotoneoPhoXiControl ]; then
  export PHOXI_CONTROL_PATH=/opt/PhotoneoPhoXiControl
  export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
  export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
fi
```
and update the environment variables by typing;
```bash
$ source ~/.bashrc
```
Finally, you can compile the ROS driver, `aist_phoxi_camera`, by typing;
```bash
$ catkin build aist_phoxi_camera
```

## Testing

You have to invoke `PhoXiControl` in advance of running
`aist_phoxi_camera`. It directly communicates
with one or more scanners. The `aist_phoxi_camera`,
establishes a connection to one of the scanners currently available on the
network, sends control commands and receives various types of 3D data,
ex. point cloud, texture map, depth map, confidence map, and etc., via
`PhoXiControl`. You can start `PhoXiControl` by typing;
```bash
$ PhoXiControl
```

Even if no real scanners are available on the network, you can test
the ROS driver by using a virtual scanner device named
"InstalledExamples-basic-example" which is provided by `PhoXiControl`.
You can launch the driver and establish a connection to the virtual
scanner by typing as follows;
```bash
$ roslaunch aist_phoxi_camera test.launch
```
If you wish to connect a real device, specify its unique ID;
```bash
$ roslaunch aist_phoxi_camera test.launch id:="2018-09-016-LC3"
```
The ID, "2018-09-016-LC3" here, varies for each device which can be
known from `Network Discovery` window of `PhoXiControl`.

By typing the command above, the ROS visualizer, `rviz`, will start as
well. Here, `rviz` is configured to subscribe two topics, `pointcloud`
and `texture`, published by `aist_phoxi_camera`. Since the driver is
started with `trigger_mode:=0`, i.e. `Free Run` mode, you will see
continuously updated pointcloud and image streams.

Another GUI, `rqt_reconfigure`, will be also started. You can dynamically
change various capturing parameters through it.

## Available ROS services

- **~/start_acquisition** -- Activate the scanner for image acquisition
- **~/stop_acquisition** -- Deactivate the scanner
- **~/trigger_frame** -- Capture an image frame and publish
- **~/get_device_list** -- Enumerate all the PhoXi 3D scanners available including the virtual scanner
- **~/get_hardware_identification** -- Show unique ID of the scanner currently connected to
- **~/get_hardware_supported_capturing_modes** -- Enumerate all the capturing modes available for the scanner currently connected to

## Published ROS topics

- **~/confidence_map** -- A 2D map of values indicating reliability of 3D measurements at each pixel
- **~/depth_map** -- A 2D map of depth values, i.e. z-coordinate values of point cloud, in meters
- **~/normal_map** -- A 2D map of surface normals
- **~/texture** -- A 2D map of intensity values. The values are in 8bit unsigned integer format
- **~/pointcloud** -- A 2D map of 3D points. Each 2D pixel has an associated intensity value in RGBA format as well as the corresponding 3D coordinates in meters
- **~/camera_info** -- Camera parameters including a 3x3 calibration matrix and lens distortions

## ROS parameters

- **~id** -- Unique ID of the scanner
- **~frame** -- Frame ID of the optical sensor stored in the headers of image and pointcloud topics
- **~trigger_mode** -- Software trigger(1) or free run(0) modes
- **~intensity_scale** -- Intensity scale of texture topic
