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
import os, re
import rospy, rospkg, tf
import xacro
import xml.dom.minidom
from tf                 import transformations as tfs
from aist_model_spawner import srv as msrv
from aist_model_spawner import msg as mmsg

#########################################################################
#  class ModelSapwnerServer                                             #
#########################################################################
class ModelSpawnerServer(object):

    _Model = '<robot name="{0}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n  <link name="{4}"/>\n  <xacro:include filename="{1}"/>\n  <xacro:{2} prefix="{3}" parent="{4}" spawn_attached="true">\n    <origin xyz="{5} {6} {7}" rpy="{8} {9} {10}"/>\n  </xacro:{2}>\n</robot>\n'

    def __init__(self, urdf_dir):
        super(ModelSpawnerServer, self).__init__()

        self._add_srv        = rospy.Service('~add', msrv.Add, self._add_cb)
        self._delete_srv     = rospy.Service('~delete', msrv.Delete,
                                             self._delete_cb)
        self._delete_all_srv = rospy.Service('~delete_all', msrv.DeleteAll,
                                             self._delete_all_cb)
        self._get_list_srv   = rospy.Service('~get_list', msrv.GetList,
                                             self._get_list_cb)
        self._urdf_dir       = urdf_dir
        self._models         = {}
        self._broadcaster    = tf.TransformBroadcaster()
        self._publisher      = rospy.Publisher('~model_description',
                                               mmsg.ModelDescription,
                                               queue_size=10)

    def tick(self):
        now = rospy.Time.now()
        for model in self._models.values():
            for childNode in model.childNodes:
                if childNode.localName != 'joint' or \
                   childNode.getAttribute('type') != 'fixed':
                    continue

                try:
                    element = childNode.getElementsByTagName('parent')[0]
                    parent  = element.getAttribute('link')
                    element = childNode.getElementsByTagName('child')[0]
                    child   = element.getAttribute('link')
                    element = childNode.getElementsByTagName('origin')[0]
                    xyz     = map(float, element.getAttribute('xyz').split())
                    rpy     = map(float, element.getAttribute('rpy').split())

                    self._broadcaster.sendTransform(
                        xyz, tfs.quaternion_from_euler(*rpy),
                        now, child, parent)
                except Exception as e:
                    rospy.logwarn(e)

    def _add_cb(self, req):
        name        = req.name
        xacro_name  = os.path.join(self._urdf_dir, name + '_macro.urdf.xacro')
        macro_name  = 'assy_part_' + re.split('[_-]', name)[0]
        position    = req.pose.pose.position
        orientation = req.pose.pose.orientation
        xacro_desc  = ModelSpawnerServer._Model.format(
                        name, xacro_name, macro_name,
                        req.prefix, req.pose.header.frame_id,
                        position.x, position.y, position.z,
                        *tfs.euler_from_quaternion([orientation.x,
                                                    orientation.y,
                                                    orientation.z,
                                                    orientation.w]))
        try:
            # Expand and process Xacro into XML format.
            doc = xacro.parse(xacro_desc)  # Create DOM tree.
            xacro.process_doc(doc)         # Expand and process macros.
            desc = doc.toprettyxml(indent='  ', encoding='utf8')
            self._models[req.prefix + name] \
                = xml.dom.minidom.parseString(desc).childNodes[0]
            self._publisher.publish(mmsg.ModelDescription.ADD,
                                    req.prefix + name, desc)
            return msrv.AddResponse(True)
        except Exception as e:
            rospy.logerr(e)
            return msrv.AddResponse(False)

    def _delete_cb(self, req):
        try:
            del self._models[req.prefix + req.name]
            self._publisher.publish(mmsg.ModelDescription.DELETE,
                                    req.prefix + req.name, '')
            return msrv.DeleteResponse(True)
        except KeyError:
            rospy.logerr('Tried to delete unknown model[' + req.prefix + req.name + '].')
            return msrv.DeleteResponse(False)

    def _delete_all_cb(self, req):
        self._models.clear()
        self._publisher.publish(mmsg.ModelDescription.DELETEALL, '', '')
        return msrv.DeleteAllResponse(True)

    def _get_list_cb(self, req):
        return msrv.GetListResponse(self._models.keys())


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('model_spawner')

    rospack  = rospkg.RosPack()
    urdf_dir = rospy.get_param('urdf_dir',
                               os.path.join(
                                   rospack.get_path('o2ac_parts_description'),
                                   'urdf/generated'))

    spawner = ModelSpawnerServer(urdf_dir)
    rate    = rospy.Rate(10)
    while not rospy.is_shutdown():
        spawner.tick()
        rate.sleep()
