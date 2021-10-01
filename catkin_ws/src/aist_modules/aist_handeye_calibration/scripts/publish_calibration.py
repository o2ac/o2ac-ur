#!/usr/bin/env python2
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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
# Author: Toshio Ueshiba
#
import rospy, tf, rosparam
from geometry_msgs            import msg as gmsg
from std_srvs                 import srv as ssrv
from aist_handeye_calibration import srv as asrv

#########################################################################
#  local functions                                                      #
#########################################################################
class CalibrationPublisher(object):
    # Dummy effector <-- camera transform in case of eye_on_hand
    Tec = {"x": 0, "y": 0, "z": 0.05, "qx": 0, "qy": 0, "qz": 0, "qw": 1}

    # Dummy base <-- camera transform in case of eye_on_base
    Tbc = {"x": 0.4, "y": 0.2, "z": 0.6,
           "qx": -0.68, "qy": -0.68, "qz": 0.22, "qw": 0.1637}

    def __init__(self):
        super(CalibrationPublisher, self).__init__()

        self._broadcaster        = tf.TransformBroadcaster()
        self._load_transform_srv = rospy.Service("~load_transform",
                                                asrv.LoadTransform,
                                                self._load_transform_cb)
        self._transform          = self._load_transform()

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._publish_transform()
            rate.sleep()

    def _load_transform_cb(self, req):
        rosparam.load_file(req.filename, '~', True)
        self._load_transform()
        return ssrv.TriggerResponse(True, "transform loaded.")

    def _load_transform(self):
        if rospy.get_param("~dummy", False):
            child  = rospy.get_param("~camera_frame")
            if rospy.get_param("~eye_on_hand", False):
                parent = rospy.get_param("~robot_effector_frame")
                T      = CalibrationPublisher.Tec
            else:
                parent = rospy.get_param("~robot_base_frame")
                T      = CalibrationPublisher.Tbc
        else:
            parent = rospy.get_param("~parent")
            child  = rospy.get_param("~child")
            T      = rospy.get_param("~transform")

        transform = gmsg.TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id  = child
        transform.transform \
            = gmsg.Transform(gmsg.Vector3(T["x"], T["y"], T["z"]),
                             gmsg.Quaternion(T["qx"], T["qy"],
                                             T["qz"], T["qw"]))
        return transform

    def _publish_transform(self):
        self._transform.header.stamp = rospy.Time.now()
        self._broadcaster.sendTransformMessage(self._transform)


#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == "__main__":
    rospy.init_node("aist_handeye_calibration_publisher")

    while rospy.get_time() == 0.0:
        pass

    cp = CalibrationPublisher()
    cp.run()
