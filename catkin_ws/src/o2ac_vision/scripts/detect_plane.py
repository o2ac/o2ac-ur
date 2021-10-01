#!/usr/bin/env python

import os
import sys
import rospy
import threading
import numpy as np
from geometry_msgs import msg as gmsg
from aist_depth_filter import DepthFilterClient
from tf import TransformBroadcaster, transformations as tfs

#########################################################################
#  gloabal functions                                                    #
#########################################################################


def transform_from_plane(plane):
    def normalize(v):
        return v / np.linalg.norm(v)

    t = gmsg.TransformStamped()
    t.header.frame_id = plane.header.frame_id
    t.child_frame_id = "tray_center"

    # Compute translation
    k = np.array([plane.plane.normal.x, plane.plane.normal.y, plane.plane.normal.z])
    x = -plane.plane.distance * k
    t.transform.translation.x = x[0]
    t.transform.translation.y = x[1]
    t.transform.translation.z = x[2]

    # Compute rotation
    j = normalize(np.cross(k, np.array([1, 0, 0])))
    i = np.cross(j, k)
    q = tfs.quaternion_from_matrix(
        np.array(
            [
                [i[0], j[0], k[0], 0.0],
                [i[1], j[1], k[1], 0.0],
                [i[2], j[2], k[2], 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    )
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t


#########################################################################
#  class PlaneDetector                                                  #
#########################################################################
class PlaneDetector(object):
    def __init__(self):
        super(PlaneDetector, self).__init__()

        self._dfilter = DepthFilterClient("depth_filter")
        self._broadcaster = TransformBroadcaster()
        self._transform = None
        self._run = True

        thread = threading.Thread(target=self._broadcast_plane)
        thread.start()

    def detect_plane(self):
        self._transform = None

    def quit(self):
        self._run = False

    def _broadcast_plane(self):
        rate = rospy.Rate(10)  # 10Hz
        while self._run:
            if self._transform is None:
                self._dfilter.detect_plane_send_goal()
                plane = self._dfilter.detect_plane_wait_for_result()
                if plane is not None:
                    self._transform = transform_from_plane(plane)

            if self._transform is not None:
                self._transform.header.stamp = rospy.Time.now()
                self._broadcaster.sendTransformMessage(self._transform)

            rate.sleep()


#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == "__main__":

    rospy.init_node("~")

    detector = PlaneDetector()

    while not rospy.is_shutdown():
        if raw_input("Hit return key >> ") == "q":
            detector.quit()
            sys.exit()
        detector.detect_plane()
