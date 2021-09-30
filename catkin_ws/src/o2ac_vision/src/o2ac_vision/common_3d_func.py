#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Chukyo University
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
#  * Neither the name of Chukyo University nor the names of its
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
# Author: Shuichi Akizuki

import open3d as o3
import numpy as np
import numpy.linalg as LA
import copy
import math


def centering(_cloud_in):
    """
    Centering()
    offset an input cloud to its centroid.

    input(s):
        _cloud_in: point cloud to be centered
    output(s):
        cloud_off:
    """
    cloud_in = copy.deepcopy(_cloud_in)
    np_m = np.asarray(cloud_in.points)
    center = np.mean(np_m, axis=0)
    np_m[:] -= center

    cloud_off = o3.geometry.PointCloud()
    cloud_off.points = o3.utility.Vector3dVector(np_m)

    return cloud_off, center


"""RPY parameters to 4x4 transformaion"""


def rpy2mat(roll, pitch, yaw):

    rot = np.identity(4)
    if roll < -3.141:
        roll += 6.282
    elif 3.141 < roll:
        roll -= 6.282
    if pitch < -3.141:
        pitch += 6.282
    elif 3.141 < pitch:
        pitch -= 6.282
    if yaw < -3.141:
        yaw += 6.282
    elif 3.141 < yaw:
        yaw -= 6.282

    rot[0, 0] = math.cos(yaw) * math.cos(pitch)
    rot[0, 1] = -math.sin(yaw) * math.cos(roll) + (
        math.cos(yaw) * math.sin(pitch) * math.sin(roll)
    )
    rot[0, 2] = math.sin(yaw) * math.sin(roll) + (
        math.cos(yaw) * math.sin(pitch) * math.cos(roll)
    )
    rot[1, 0] = math.sin(yaw) * math.cos(pitch)
    rot[1, 1] = math.cos(yaw) * math.cos(roll) + (
        math.sin(yaw) * math.sin(pitch) * math.sin(roll)
    )
    rot[1, 2] = -math.cos(yaw) * math.sin(roll) + (
        math.sin(yaw) * math.sin(pitch) * math.cos(roll)
    )
    rot[2, 0] = -math.sin(pitch)
    rot[2, 1] = math.cos(pitch) * math.sin(roll)
    rot[2, 2] = math.cos(pitch) * math.cos(roll)
    rot[3, 0] = rot[3, 1] = rot[3, 2] = 0.0
    rot[3, 3] = 1.0

    return rot


"""Convert matrix to RPY parameters"""


def mat2rpy(rot):
    roll = math.atan2(rot[2, 1], rot[2, 2])
    pitch = math.atan2(
        -rot[2, 0], math.sqrt(rot[2, 1] * rot[2, 1] + rot[2, 2] * rot[2, 2])
    )
    yaw = math.atan2(rot[1, 0], rot[0, 0])

    return roll, pitch, yaw
