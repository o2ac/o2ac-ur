#!/usr/bin/env python

import rospy
from aist_depth_filter       import DepthFilterClient
from aist_camera_multiplexer import RealSenseMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('~')

    camera_names = rospy.get_param('camera_multiplexer/camera_names', [])
    for camera_name in camera_names:
        filter = DepthFilterClient(camera_name + '/depth_filter')
        filter.window_radius = 2

    multiplexer = RealSenseMultiplexerClient('camera_multiplexer')

    while not rospy.is_shutdown():
        print camera_names
        try:
            camera_number = int(raw_input('Camera number >> '))
            if camera_number < 0 or camera_number >= len(camera_names):
                break
            multiplexer.activate_camera(camera_names[camera_number])
        except Exception as e:
            print e
