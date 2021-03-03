#!/usr/bin/env python

import rospy
import sensor_msgs.msg

import os
import cv2
import cv_bridge
import numpy as np
import copy
import time


class O2ACCameraHelper(object):
    """
    A class to help deal with vision calculations and camera management.
    Stores the camera_info of the currently active camera,
    offers functions to convert points from 2D to 3D.
    """

    def __init__(self):
        self.camera_name = "camera_multiplexer"
        self.cam_info_sub = rospy.Subscriber('/' + self.camera_name + '/camera_info', sensor_msgs.msg.CameraInfo,
                                          self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber('/' + self.camera_name + '/depth', sensor_msgs.msg.Image,
                                          self.depth_image_sub_callback)
        
        self._camera_info = []
        self._depth_image = []

        self.bridge = cv_bridge.CvBridge()

    def camera_info_callback(self, cam_info_message):
        self._camera_info = cam_info_message
    
    def depth_image_sub_callback(self, depth_image_msg):
        self._depth_image = depth_image_msg

    def project_2d_to_3d_with_current_view(self, u, v, frames_to_average=1, average_with_radius=0):
        """
        Look at current depth image (stream) and return 3D pose of single pixel (u,v).
        frames_to_average is the number of frames used to take the average of.
        average_with_radius uses nearby pixels for averaging the depth value. radius=0 evaluates only the pixel.
        """
        
        # Get depth images
        depth_images_ros = []
        for i in range(frames_to_average):
            depth_images_ros.append(copy.deepcopy(self._depth_image))
            rospy.sleep(.12)
        
        # Convert images
        depth_images = []
        for img_ros in depth_images_ros:
            depth_images.append(self.bridge.imgmsg_to_cv2(img_ros, desired_encoding="passthrough"))
        
        # TODO: Insert fallback with increased tolerance if depth values are empty?
        return self.project_2d_to_3d_from_images(u, v, depth_images, average_with_radius)
        
    def project_2d_to_3d_from_images(self, u, v, depth_images=[], average_with_radius=0):
        """
        Go through depth images (list of images in cv2 format) and return 3D pose of single pixel (u,v).
        average_with_radius uses nearby pixels for averaging the depth value. radius=0 evaluates only the pixel.
        """
        # Average over images and area
        depth_vals = []
        for img in depth_images:
            if img[v, u]:  # Skip if pixel empty
                depth_vals.append(img[v, u])
            else:
                rospy.logwarn("Skipping pixel in depth backprojection")
            if average_with_radius:
                for i in range(-average_with_radius, average_with_radius):
                    for j in range(-average_with_radius, average_with_radius):
                        uc = int(np.clip(estimated_poses_msg.bbox[1] + round(estimated_poses_msg.bbox[3]/2), 0, 479))
                        vc = int(np.clip(estimated_poses_msg.bbox[1] + round(estimated_poses_msg.bbox[3]/2), 0, 479))
                        depth_vals.append(img[vc, uc])
        
        if len(depth_vals) == 0:
            depth_vals.append(0)
            rospy.logerr("Depth is zero!")
        depth = np.mean(depth_vals)
        
        # Backproject to 3D
        position = self.project_2d_to_3d([u], [v], [depth])
        return position[0]

    def project_2d_to_3d(self, u, v, d):
        """
        Back-project 2D image points to 3D space using depth values.
        u, v are the image-space coordinates of the depth values d.
        u, v, d need to be lists.
        """
        npoints = len(d)
        xy = cv2.undistortPoints(np.expand_dims(np.array(list(zip(u, v)),
                                                            dtype=np.float32),
                                                axis=0),
                                    np.array(self._camera_info.K).reshape((3, 3)),
                                    np.array(self._camera_info.D))
        xy = xy.ravel().reshape(npoints, 2)
        return [ (xy[i, 0]*d[i], xy[i, 1]*d[i], d[i])
                    for i in range(npoints) ]    
