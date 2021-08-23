#!/usr/bin/env python

import sys, rospy
from aist_camera_multiplexer import (CameraMultiplexerClient,
                                     RealSenseMultiplexerClient)

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    camera_name = sys.argv[1]

    rospy.init_node('~')

    if rospy.get_param("use_real_robot", False):
        multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
    else:
        multiplexer = CameraMultiplexerClient('camera_multiplexer')

    multiplexer.activate_camera(camera_name)
