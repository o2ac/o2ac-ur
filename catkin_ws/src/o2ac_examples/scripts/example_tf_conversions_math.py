#!/usr/bin/env python

import numpy as np

import rospy
import tf
import tf_conversions
from math import *
import geometry_msgs.msg


def phoxi_transformation_examples():
    # original matrix
    T_phoxi = np.array([
        [-0.034784670752, 0.873298269909, -0.485942546454, 0.594404677631629],
        [0.979858695089, -0.065869488303, -0.188515644360, 0.128504467275411],
        [-0.196639172950, -0.482712484078, -0.853417654714, 1.427249342095321],
        [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
    ])

    # https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
    # Example code for getting rpy angles and translation from the 4x4
    # homogenous matrix via TF
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(
        T_phoxi)
    print("angles: {}".format(angles))
    print("trans: {}".format(trans))

    # rpy = tf.transformations.euler_from_matrix(T_phoxi)
    # xyz = [T_phoxi[0,3], T_phoxi[1,3], T_phoxi[2,3]]
    # print(rpy)
    # print(xyz)

    # T_phoxi is the transformation to the camera frame
    # The transformation to be entered into the URDF is to the camera base
    # frame

    # One possibility: Take the transformation between camera frame and camera base_frame from the URDF,
    # and add it to the transformation here manually.

    rpy_camera_frame_to_base = [pi / 2, pi / 2, pi / 2]
    xyz_camera_frame_to_base = [1, 2, 3]
    T_camera_frame_to_base = tf.transformations.compose_matrix(
        scale=None,
        shear=None,
        angles=rpy_camera_frame_to_base,
        translate=xyz_camera_frame_to_base,
        perspective=None)

    T_phoxi_base = tf.transformations.concatenate_matrices(
        T_phoxi, T_camera_frame_to_base)
    # Now rpy and xyz can be extracted for the T_phoxi_base transformation and used in the scene URDF,
    # as the transformation from world to camera base.
    # NOTE: The signs and multiplications in the code above may have to be
    # inverted.

    return


def quaternion_eular_test():
    q = geometry_msgs.msg.Quaternion(-0.5, 0.5, 0.5, 0.5)
    rpy = tf.transformations.euler_from_quaternion([-0.5, 0.5, 0.5, 0.5])
    print(rpy)

    return


if __name__ == '__main__':
    phoxi_transformation_examples()
    # quaternion_eular_test()
