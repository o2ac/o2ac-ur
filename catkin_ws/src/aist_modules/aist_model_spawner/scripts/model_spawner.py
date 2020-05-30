#!/usr/bin/env python

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
                        '', req.pose.header.frame_id,
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
            self._models[name] = xml.dom.minidom.parseString(desc).childNodes[0]
            self._publisher.publish(mmsg.ModelDescription.ADD, name, desc)
            return msrv.AddResponse(True)
        except Exception as e:
            rospy.logerr(e)
            return msrv.AddResponse(False)

    def _delete_cb(self, req):
        try:
            del self._models[req.name]
            self._publisher.publish(mmsg.ModelDescription.DELETE, req.name, '')
            return msrv.DeleteResponse(True)
        except KeyError:
            rospy.logerr('Tried to delete unknown model[' + req.name + '].')
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
    rospy.init_node('model_spwner')

    rospack  = rospkg.RosPack()
    urdf_dir = rospy.get_param('urdf_dir',
                               os.path.join(
                                   rospack.get_path('o2ac_parts_description'),
                                   'urdf/generated'))

    spawner = ModelSpawnerServer(urdf_dir)
    rate    = rospy.Rate(1)
    while not rospy.is_shutdown():
        spawner.tick()
        rate.sleep()
