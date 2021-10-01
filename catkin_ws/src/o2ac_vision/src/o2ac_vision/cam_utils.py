#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X Corp.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski, Toshio Ueshiba

import rospy
import sensor_msgs.msg

import os
import cv2
import cv_bridge
import numpy as np
import copy
import time

CAMERA_FAILURE = -1


class O2ACCameraHelper(object):
    """
    A class to help deal with vision calculations and camera management.
    Stores the camera_info of the currently active camera,
    offers functions to convert points from 2D to 3D.
    """

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

    def project_2d_to_3d_with_current_view(
        self, camera_info, depth_image, u, v, frames_to_average=1, average_with_radius=0
    ):
        """
        Look at current depth image (stream) and return 3D pose of single pixel (u,v).
        frames_to_average is the number of frames used to take the average of.
        average_with_radius uses nearby pixels for averaging the depth value. radius=0 evaluates only the pixel.
        """

        # Get depth images
        depth_images_ros = []
        for i in range(frames_to_average):
            depth_images_ros.append(copy.deepcopy(depth_image))
            rospy.sleep(0.12)

        # Convert images
        depth_images = []
        for img_ros in depth_images_ros:
            depth_images.append(
                self.bridge.imgmsg_to_cv2(img_ros, desired_encoding="passthrough")
            )

        return self.project_2d_to_3d_from_images(
            camera_info, u, v, depth_images, average_with_radius
        )

    def get_depth_vals_from_radius(self, img, u, v, average_with_radius=5):
        """Does not actually use a radius, but a box around the pixel."""
        depth_vals = []
        if average_with_radius:
            for i in range(-average_with_radius, average_with_radius):
                for j in range(-average_with_radius, average_with_radius):
                    try:
                        uc = int(np.clip(u + i, 0, 640))
                        vc = int(np.clip(v + j, 0, 480))
                        depth_vals.append(img[vc, uc])
                    except BaseException:
                        rospy.logwarn("Error in get_depth_vals_from_radius")
                        pass
        return depth_vals

    def project_2d_to_3d_from_images(
        self, camera_info, u, v, depth_images=[], average_with_radius=0
    ):
        """
        Go through depth images (list of images in cv2 format) and return 3D pose of single pixel (u,v).
        average_with_radius uses nearby pixels for averaging the depth value. radius=0 evaluates only the pixel.
        Does not actually use a radius, but a box around the pixel.
        """
        # Average over images and area
        depth_vals = []
        try:
            for img in depth_images:
                if img[v, u]:  # Skip if pixel empty
                    depth_vals.append(img[v, u])
                else:
                    rospy.logwarn("Skipping pixel in depth backprojection")
                if average_with_radius:
                    depth_vals.extend(
                        self.get_depth_vals_from_radius(img, u, v, average_with_radius)
                    )
        except BaseException:
            pass

        if not depth_vals:
            if not average_with_radius:
                rospy.loginfo("Reattempting backprojection with neighboring pixels")
                return self.project_2d_to_3d_from_images(
                    camera_info, u, v, depth_images, average_with_radius=8
                )
            else:
                rospy.logerr(
                    "Could not find pixels in depth image to reproject! Returning interpolated"
                )
                # Use crudely interpolated depth
                d_center = np.mean(
                    self.get_depth_vals_from_box(
                        depth_images[0], 320, 240, average_with_radius=5
                    )
                )
                # 42: Hypotenuse. 37: Dist to tray at high view. 20: dist to
                # tray edge (other leg of the triangle)
                x_factor = np.round(abs(u - 320) * 42.0 / 37.0)
                y_factor = np.round(abs(v - 240) * 39.0 / 37.0)

                d_interpolated = d_center * (1 + (x_factor + y_factor) / 2.0)
                print("d_interpolated", d_interpolated)
                depth_vals.append(d_center)

        depth = np.mean(depth_vals)

        if average_with_radius:
            print(
                "With average_with_radius, found "
                + str(len(depth_vals))
                + " depth vals with mean of "
                + str(depth)
            )

        # Backproject to 3D
        position = self.project_2d_to_3d(camera_info, [u], [v], [depth])
        # if position.z
        return position[0]

    def project_2d_to_3d(self, camera_info, u, v, d):
        """
        Back-project 2D image points to 3D space using depth values.
        u, v are the image-space coordinates of the depth values d.
        u, v, d need to be lists.
        """
        npoints = len(d)
        xy = cv2.undistortPoints(
            np.expand_dims(np.array(list(zip(u, v)), dtype=np.float32), axis=0),
            np.array(camera_info.K).reshape((3, 3)),
            np.array(camera_info.D),
        )
        xy = xy.ravel().reshape(npoints, 2)
        return [(xy[i, 0] * d[i], xy[i, 1] * d[i], d[i]) for i in range(npoints)]
