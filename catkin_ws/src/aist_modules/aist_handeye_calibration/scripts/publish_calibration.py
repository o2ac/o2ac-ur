#!/usr/bin/env python2

import os
import rospy
from tf import TransformBroadcaster, TransformListener, transformations as tfs
from geometry_msgs import msg as gmsg
from math import degrees

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

        self._broadcaster = TransformBroadcaster()
        self._listener    = TransformListener()

        eye_on_hand = rospy.get_param("~eye_on_hand", False)
        parent = rospy.get_param("~robot_effector_frame" if eye_on_hand else
                                 "~robot_base_frame")
        child  = rospy.get_param("~camera_frame")

        self._dummy = rospy.get_param("~dummy", False)
        if not self._dummy:
            T = rospy.get_param("~transform")
        elif eye_on_hand:
            T = CalibrationPublisher.Tec
        else:
            T = CalibrationPublisher.Tbc

        self._transform = gmsg.TransformStamped()
        self._transform.header.frame_id = parent
        self._transform.child_frame_id  = child
        self._transform.transform \
            = gmsg.Transform(gmsg.Vector3(T["x"], T["y"], T["z"]),
                             gmsg.Quaternion(
                                 T["qx"], T["qy"], T["qz"], T["qw"]))

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if not self._dummy:
            # Get camera(optical) <- camera(body) transform
            opt_body = self._get_transform(
                        rospy.get_param("~camera_optical_frame"),
                        rospy.get_param("~camera_body_frame"))
            bot_opt  = ((self._transform.transform.translation.x,
                         self._transform.transform.translation.y,
                         self._transform.transform.translation.z),
                        (self._transform.transform.rotation.x,
                         self._transform.transform.rotation.y,
                         self._transform.transform.rotation.z,
                         self._transform.transform.rotation.w))

            mat = tfs.concatenate_matrices(
                self._listener.fromTranslationRotation(*bot_opt),
                self._listener.fromTranslationRotation(*opt_body))
            print("\n=== Estimated effector/base <- camera_body transform ===")
            self._print_mat(mat)
            print("\n")
        return True

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._transform.header.stamp = rospy.Time.now()
            self._broadcaster.sendTransformMessage(self._transform)
            rate.sleep()

    def _get_transform(self, target_frame, source_frame):
        now = rospy.Time.now()
        self._listener.waitForTransform(target_frame, source_frame, now,
                                        rospy.Duration(10))
        return self._listener.lookupTransform(target_frame, source_frame, now)

    def _print_mat(self, mat):
        xyz = tfs.translation_from_matrix(mat)
        rpy = map(degrees, tfs.euler_from_matrix(mat))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))

    def _print_transform(self, transform):
        xyz = (transform.translation.x,
               transform.translation.y, transform.translation.z)
        rpy = map(degrees, tfs.euler_from_quaternion((transform.rotation.x,
                                                      transform.rotation.y,
                                                      transform.rotation.z,
                                                      transform.rotation.w)))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))


#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == "__main__":
    rospy.init_node("aist_handeye_calibration_publisher")

    while rospy.get_time() == 0.0:
        pass

    with CalibrationPublisher() as cp:
        cp.run()
