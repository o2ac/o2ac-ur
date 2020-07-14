import rospy
from aist_model_spawner import srv as msrv

#########################################################################
#  class ModelSpawnerClient                                             #
#########################################################################
class ModelSpawnerClient(object):
    def __init__(self, server='model_spawner'):
        super(ModelSpawnerClient, self).__init__()

        self._add        = rospy.ServiceProxy(server + '/add',    msrv.Add)
        self._delete     = rospy.ServiceProxy(server + '/delete', msrv.Delete)
        self._delete_all = rospy.ServiceProxy(server + '/delete_all',
                                              msrv.DeleteAll)
        self._get_list   = rospy.ServiceProxy(server + '/get_list',
                                              msrv.GetList)

    def add(self, name, pose):
        return self._add(name, pose).success

    def delete(self, name):
        return self._delete(name).success

    def delete_all(self):
        return self._delete_all().success

    def get_list(self):
        return self._get_list().names
