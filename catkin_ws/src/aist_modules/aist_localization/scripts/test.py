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
import rospy, argparse
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test localizeAction')
    parser.add_argument('-m',
                        '--model',
                        action='store',
                        nargs='?',
                        default='04_37D-GEARMOTOR-50-70',
                        type=str,
                        choices=None,
                        help='name of model to be matched',
                        metavar=None)
    parser.add_argument('-s',
                        '--scene',
                        action='store',
                        nargs='?',
                        default='scene.ply',
                        type=str,
                        choices=None,
                        help='path to scene PLY file',
                        metavar=None)
    parser.add_argument('-f',
                        '--frame',
                        action='store',
                        nargs='?',
                        default='world',
                        type=str,
                        choices=None,
                        help='camera frame',
                        metavar=None)
    parser.add_argument('-n',
                        '--nposes',
                        action='store',
                        nargs='?',
                        default=2,
                        type=int,
                        choices=None,
                        help='the number of candidate poses',
                        metavar=None)
    parser.add_argument('-t',
                        '--timeout',
                        action='store',
                        nargs='?',
                        default=10,
                        type=float,
                        choices=None,
                        help='timeout value',
                        metavar=None)
    args = parser.parse_args()

    rospy.init_node('localization_client')

    localize = LocalizationClient()
    localize.load_scene(args.scene, args.frame)
    models  = rospy.get_param('aist_localization', [])
    spawner = ModelSpawnerClient()


    for model in models:
        localize.send_goal(model, args.nposes)
        (poses, overlaps, success) \
            = localize.wait_for_result(rospy.Duration(args.timeout))

        if poses:
            print('{}\noverlap: {}'.format(poses[0], overlaps[0]))
            spawner.add(model, poses[0])

        # for pose, overlap in zip(poses, overlaps):
        #     print('{}\noverlap: {}'.format(pose, overlap))
        #     spawner.add(model, pose)

    print(spawner.get_list())
