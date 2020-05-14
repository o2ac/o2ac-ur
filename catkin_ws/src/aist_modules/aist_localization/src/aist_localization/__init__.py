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
        self._load_plcf = rospy.ServiceProxy(name + "/load_plcf",
                                             lsrv.LoadModel)
        self._save_plcf = rospy.ServiceProxy(name + "/save_plcf", ssrv.Trigger)
        self._load_ply  = rospy.ServiceProxy(name + "/load_ply",
                                             lsrv.LoadModel)
        self._localize  = actionlib.SimpleActionClient(name + "/localize",
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

    def send_goal(self, model, number_of_poses=1):
        self._poses    = []
        self._overlaps = []
        goal = lmsg.LocalizeGoal()
        goal.object_name     = model
        goal.number_of_poses = number_of_poses
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self, timeout=rospy.Duration()):
        if (not self._localize.wait_for_result(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired.

        # Sort obtained poses in descending order of overlap values.
        pairs = sorted(zip(self._poses, self._overlaps),
                       key=itemgetter(1), reverse=True)
        if len(pairs):
            self._poses, self._overlaps = zip(*pairs)

        return (self._poses, self._overlaps)

    def _feedback_cb(self, feedback):
        self._poses.append(feedback.pose)
        self._overlaps.append(feedback.overlap)
