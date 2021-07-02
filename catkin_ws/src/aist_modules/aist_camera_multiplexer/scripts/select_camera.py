#!/usr/bin/env python

import sys, rospy
from aist_camera_multiplexer import CameraMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    camera_name = sys.argv[1]

    rospy.init_node('~')
    multiplexer = CameraMultiplexerClient('camera_multiplexer')
    multiplexer.activate_camera(camera_name)
