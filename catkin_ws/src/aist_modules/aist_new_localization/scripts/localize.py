#!/usr/bin/env python
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
import rospy, sys, re
from aist_depth_filter     import DepthFilterClient
from aist_new_localization import LocalizationClient
from aist_model_spawner    import ModelSpawnerClient

if __name__ == '__main__':

    rospy.init_node('~')
    nposes    = rospy.get_param('~nposes',  2)
    timeout   = rospy.get_param('~timeout', 10)
    models    = rospy.get_param('~models',  [])

    dfilter   = DepthFilterClient('depth_filter')
    dfilter.window_radius = 2
    localizer = LocalizationClient('localization')
    spawner   = ModelSpawnerClient('/model_spawner')

    while not rospy.is_shutdown():
        print('\nmodels: {}\n'.format(models))
        try:
            key = raw_input('Model # >> ')
            if key == 'q':
                break
            num = int(key)
            model = [m for m in models if int(re.split('[_-]', m)[0]) == num][0]

            spawner.delete_all()
            dfilter.capture()                   # Load PLY to the localizer
            localizer.send_goal(model, nposes)
            (poses, overlaps) \
                = localizer.wait_for_result(rospy.Duration(timeout))

            print('{} poses found. Overlaps are {}.'
                  .format(len(poses), overlaps))

            for pose in reversed(poses):
                spawner.add(model, pose)
                rospy.sleep(1)

        except ValueError:
            print('Please specify model number.')
        except KeyboardInterrupt:
            sys.exit()
