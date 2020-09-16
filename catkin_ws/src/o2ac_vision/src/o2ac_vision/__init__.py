import rospy
import actionlib
from o2ac_msgs import msg as omsg

#########################################################################
#  class PoseEstimationClient                                           #
#########################################################################
class PoseEstimationClient(object):
    def __init__(self, server='o2ac_vision'):
        super(PoseEstimationClient, self).__init__()
        self._client \
            = actionlib.SimpleActionClient(server + '/poseEstimationTest',
                                           omsg.poseEstimationTestAction)
        self._client.wait_for_server()

    def trigger(self, object_id=1, camera_id=1):
        goal = omsg.poseEstimationTestGoal(object_id=object_id,
                                           camera_id=camera_id)
        self._client.send_goal(goal)

    def get_results(self, timeout=rospy.Duration()):
        if (not self._client.wait_for_result(timeout)):
            self._client.cancel_goal()  # Cancel if timeout expired

        return _client.get_result().pose_estimation_result_list

#########################################################################
#  class BeltDetectionClient                                            #
#########################################################################
class BeltDetectionClient(object):
    def __init__(self, server='o2ac_vision'):
        super(BeltDetectionClient, self).__init__()
        self._clinet \
            = actionlib.SimpleActionClient(server + '/beltDetectionTest',
                                           omsg.beltDetectionTestAction)
        self._client.wait_for_server()

    def trigger(self, object_id=1, camera_id=1):
        goal = omsg.beltDetectionTestGoal(object_id=object_id,
                                          camera_id=camera_id)
        self._client.send_goal(goal)

    def get_grasp_points(self, timeout=rospy.Duration()):
        if (not self._client.wait_for_result(timeout)):
            self._client.cancel_goal()  # Cancel if timeout expired

        return _client.get_result().grasp_points
