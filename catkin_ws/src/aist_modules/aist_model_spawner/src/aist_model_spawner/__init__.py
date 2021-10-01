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
from aist_model_spawner import srv as msrv

#########################################################################
#  class ModelSpawnerClient                                             #
#########################################################################
class ModelSpawnerClient(object):
    def __init__(self, server='model_spawner'):
        super(ModelSpawnerClient, self).__init__()

        rospy.wait_for_service(server + '/add')
        rospy.wait_for_service(server + '/delete')
        rospy.wait_for_service(server + '/delete_all')
        rospy.wait_for_service(server + '/get_list')

        self._add        = rospy.ServiceProxy(server + '/add',    msrv.Add)
        self._delete     = rospy.ServiceProxy(server + '/delete', msrv.Delete)
        self._delete_all = rospy.ServiceProxy(server + '/delete_all',
                                              msrv.DeleteAll)
        self._get_list   = rospy.ServiceProxy(server + '/get_list',
                                              msrv.GetList)

    def add(self, name, pose, prefix=''):
        return self._add(name, prefix, pose).success

    def delete(self, name, prefix=''):
        return self._delete(name, prefix).success

    def delete_all(self):
        return self._delete_all().success

    def get_list(self):
        return self._get_list().names
