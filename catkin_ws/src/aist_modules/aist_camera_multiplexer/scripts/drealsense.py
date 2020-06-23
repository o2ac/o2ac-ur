#!/usr/bin/env python

import rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('~')

    camera_names = rospy.get_param('camera_multiplexer/camera_names', [])
    multiplexer  = RealSenseMultiplexerClient('camera_multiplexer')

    while not rospy.is_shutdown():
        for camera_number, camera_name in enumerate(camera_names):
            print('{}: {}'.format(camera_number, camera_name))

        try:
            camera_number = int(raw_input('Camera number >> '))
            if camera_number < 0 or camera_number >= len(camera_names):
                break
            multiplexer.activate_camera(camera_names[camera_number])
        except Exception as e:
            print e
