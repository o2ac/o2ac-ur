import rospy
from aist_model_spawner import srv as msrv

#########################################################################
#  class ModelSpawnerClient                                             #
#########################################################################
class ModelSpawnerClient(object):
    def __init__(self, name="model_spawner"):
        super(ModelSpawnerClient, self).__init__()

        self._add        = rospy.ServiceProxy(name + "/add",    msrv.Add)
        self._delete     = rospy.ServiceProxy(name + "/delete", msrv.Delete)
        self._delete_all = rospy.ServiceProxy(name + "/delete_all",
                                              msrv.DeleteAll)
        self._get_list   = rospy.ServiceProxy(name + "/get_list", msrv.GetList)

    def add(self, name, pose):
        return self._add(name, pose).success

    def delete(self, name):
        return self._delete(name).success

    def delete_all(self):
        return self._delete_all().success

    def get_list(self):
        return self._get_list().names
