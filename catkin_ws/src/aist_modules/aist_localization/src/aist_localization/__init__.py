import rospy
import dynamic_reconfigure.client
import actionlib
from std_srvs          import srv as ssrv
from aist_localization import msg as lmsg, srv as lsrv
from operator          import itemgetter

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    def __init__(self, name="localization"):
        super(LocalizationClient, self).__init__()

        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._load_plcf = rospy.ServiceProxy(name + "/load_plcf", lsrv.LoadPlcf)
        self._save_plcf = rospy.ServiceProxy(name + "/load_plcf", ssrv.Trigger)
        self._localize  = actionlib.SimpleActionClient(name + "/localize",
                                                       lmsg.LocalizeAction)
        self._localize.wait_for_server()

    def set_setting(self, name, value):
        self._dyn_reconf.update_configuration({name : value})
        return self.get_setting(name)

    def get_setting(self, name):
        return self.get_settings()[name]

    def get_settings(self):
        return self._dyn_reconf.get_configuration()

    def load_config(self, object_name):
        return self._load_plcf(object_name).success

    def save_config(self, object_name):
        return self._save_plcf().success

    def send_goal(self, number_of_poses=1):
        self._poses    = []
        self._overlaps = []
        goal = lmsg.LocalizeGoal()
        goal.number_of_poses = number_of_poses
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=5):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired.

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        result = self._localize.get_result()
        return (self._poses, self._overlaps,
                result.success if result else False)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)
