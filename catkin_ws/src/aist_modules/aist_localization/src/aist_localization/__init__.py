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
import rospy
import dynamic_reconfigure.client
import actionlib
from geometry_msgs     import msg as gmsg
from aist_localization import msg as lmsg
from operator          import itemgetter

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    _DefaultSettings = {
                           'Scene_Clustering_Level':              'Normal',
                           'Scene_Minimal_Cluster_Size':          200,
                           'Scene_Maximal_Cluster_Size':          3500000,
                           'Matching_Algorithm':                  'Surfaces',
                           'Model_Keypoints_Sampling':            'Medium',
                           'Local_Search_Radius':                 'Normal',
                           'Feature_fit_consideration_level':     15,
                           'Global_maximal_feature_fit_overflow': 20,
                           'Fine_Alignment_Iterations':           30,
                           'Fine_Alignment_Point_Set':            'Surface',
                           'Fine_Alignment_Point_Set_Sampling':   'Sampled',
                           'Projection_Tolerance':                100,
                           'Projection_Hidden_Part_Tolerance':    100,
                           'Overlap':                             15.0
                       }

    def __init__(self, server='localization'):
        super(LocalizationClient, self).__init__()

        self._in_plane          = rospy.get_param('~in_plane',          {})
        self._full_settings     = rospy.get_param('~full_settings',     {})
        self._in_plane_settings = rospy.get_param('~in_plane_settings', {})

        self._dyn_reconf = dynamic_reconfigure.client.Client(server,
                                                             timeout=5.0)
        self._localize   = actionlib.SimpleActionClient(server + '/localize',
                                                        lmsg.LocalizeAction)
        self._localize.wait_for_server()

    def set_setting(self, name, value):
        self.set_settings({name : value})
        return self.get_setting(name)

    def get_setting(self, name):
        return self.get_settings()[name]

    def set_settings(self, settings):
        self._dyn_reconf.update_configuration(settings)

    def get_settings(self):
        return self._dyn_reconf.get_configuration()

    def send_goal(self, model, nposes=1, poses2d=[]):
        goal = lmsg.LocalizeGoal()
        goal.object_name = model
        goal.in_plane    = model in self._in_plane
        goal.nposes      = nposes
        goal.poses2d     = poses2d
        if goal.in_plane and not goal.poses2d:
            goal.poses2d = [gmsg.Pose2D(x=0.0, y=0.0, theta=0.0)]
        goal.sideways    = False
        goal.x_offset    = 0.0
        goal_z_offset    = 0.0
        if model in self._in_plane_settings:
            in_plane_settings = self._in_plane_settings[model]
            if 'sideways' in in_plane_settings:
                goal.sideways = in_plane_settings['sideways']
            if 'x_offset' in in_plane_settings:
                goal.x_offset = in_plane_settings['x_offset']
            if 'z_offset' in in_plane_settings:
                goal.z_offset = in_plane_settings['z_offset']

        self.set_settings(LocalizationClient._DefaultSettings)
        if model in self._full_settings:
            self.set_settings(self._full_settings[model])

        self._poses    = []
        self._overlaps = []
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=rospy.Duration()):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        return (self._poses, self._overlaps)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)
