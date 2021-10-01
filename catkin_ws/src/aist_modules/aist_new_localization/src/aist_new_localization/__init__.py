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
import rospy, copy
import dynamic_reconfigure.client
import actionlib
import numpy as np
from tf                    import transformations as tfs
from geometry_msgs         import msg as gmsg
from aist_new_localization import msg as lmsg
from aist_depth_filter     import msg as dmsg
from operator              import itemgetter
from actionlib_msgs.msg    import GoalStatus

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    _DefaultParams = {'origin':           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      'rotation_range':   [0.0, 0.0, 0,0],
                      'refine_transform': False,
                      'check_borders':    False}

    def __init__(self, server='/localization'):
        super(LocalizationClient, self).__init__()

        self._params     = rospy.get_param('~localization_parameters',  {})
        self._dyn_reconf = dynamic_reconfigure.client.Client(server,
                                                             timeout=5.0)
        self._localize   = actionlib.SimpleActionClient(server + '/localize',
                                                        lmsg.LocalizeAction)

        for part, params in self._params.items():
            for key, value in LocalizationClient._DefaultParams.items():
                if key not in params:
                    self._params[part][key] = value
        print(self._params)
        print('start localizer...')
        self._localize.wait_for_server()
        print('OK')

    def set_setting(self, name, value):
        self.set_settings({name : value})
        return self.get_setting(name)

    def get_setting(self, name):
        return self.get_settings()[name]

    def set_settings(self, settings):
        self._dyn_reconf.update_configuration(settings)

    def get_settings(self):
        return self._dyn_reconf.get_configuration()

    def send_goal(self, object_name, plane, poses2d):
        params = self._params[object_name]
        origin = copy.copy(params['origin'])  # Must be copied
        origin[3] = np.radians(origin[3])
        origin[4] = np.radians(origin[4])
        origin[5] = np.radians(origin[5])
        goal = lmsg.LocalizeGoal()
        rotation_range = params['rotation_range']
        if rotation_range[0] < rotation_range[1] and \
           rotation_range[2] > 0:
            goal.poses2d = [ gmsg.Pose2D(poses2d[0].x,
                                         poses2d[0].y,
                                         np.radians(theta) + poses2d[0].theta)
                             for theta in np.arange(*rotation_range) ]
        else:
            goal.poses2d = poses2d
        goal.object_name = object_name
        goal.plane       = plane
        goal.origin      = gmsg.Pose(gmsg.Point(*origin[0:3]),
                                     gmsg.Quaternion(*tfs.quaternion_from_euler(
                                         *origin[3:6])))
        goal.refine_transform = params['refine_transform']
        goal.check_borders    = params['check_borders']
        self._localize.send_goal(goal)

    def send_goal_with_target_frame(self, object_name, frame_id, stamp,
                                    poses2d):
        plane = dmsg.PlaneStamped()
        plane.header.frame_id = frame_id
        plane.header.stamp    = stamp
        plane.plane.normal.x  = 0
        plane.plane.normal.y  = 0
        plane.plane.normal.z  = 1
        plane.plane.distance  = 0
        return self.send_goal(object_name, plane, poses2d)

    def wait_for_result(self, timeout=0):
        if not self._localize.wait_for_result(rospy.Duration(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired
            return None
        elif self._localize.get_state() != GoalStatus.SUCCEEDED:
            return None
        return self._localize.get_result()
