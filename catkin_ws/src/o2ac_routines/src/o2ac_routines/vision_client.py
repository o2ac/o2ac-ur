import actionlib
import rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient
import o2ac_msgs.msg

class VisionClient():
    def __init__(self):

        try:
            self.vision_multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
        except:
            self.vision_multiplexer = []

        self.ssd_client = actionlib.SimpleActionClient('/o2ac_vision_server/get_3d_poses_from_ssd', o2ac_msgs.msg.get3DPosesFromSSDAction)
        self.detect_shaft_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_shaft_notch', o2ac_msgs.msg.shaftNotchDetectionAction)
        self.detect_angle_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_angle', o2ac_msgs.msg.detectAngleAction)
        self.localization_client = actionlib.SimpleActionClient('/o2ac_vision_server/localize_object', o2ac_msgs.msg.localizeObjectAction)

    def activate_camera(self, camera_name="b_bot_outside_camera"):
        try:
            if self.vision_multiplexer:
                return self.vision_multiplexer.activate_camera(camera_name)
            else:
                rospy.logwarn("Camera multiplexer not functional! Returning true")
                return True
        except:
            pass
        rospy.logwarn("Could not activate camera! Returning false")
        return False

    def read_from_sdd(self):
        """
        Returns object poses as estimated by the SSD neural network and reprojection.
        Also updates self.objects_in_tray
        """
        # Send goal, wait for result
        self.ssd_client.send_goal(o2ac_msgs.msg.get3DPosesFromSSDGoal())
        if (not self.ssd_client.wait_for_result(rospy.Duration(4.0))):
            self.ssd_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call for SSD result returned no result. Is o2ac_vision running?")
            return False

        # Read result and return
        try:
            return self.ssd_client.get_result()
        except:
            pass
        return False

    def get_angle_from_vision(self, camera="b_bot_inside_camera", item_name="bearing"):
        # Send goal, wait for result
        goal = o2ac_msgs.msg.detectAngleGoal()
        goal.item_id = item_name
        goal.camera_id = camera
        self.detect_angle_client.send_goal(goal)
        if (not self.detect_angle_client.wait_for_result(rospy.Duration(3.0))):
            self.detect_angle_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to detect angle returned no result. Is o2ac_vision running?")
            return False

        # Read result and return
        try:
            res = self.detect_angle_client.get_result()
            if res.succeeded:
                return res.rotation_angle
        except:
            pass
        return False

    def call_shaft_notch_detection(self):
        """
        Calls the action and returns the result as is
        """
        goal = o2ac_msgs.msg.shaftNotchDetectionGoal()
        self.detect_shaft_client.send_goal(goal)
        if (not self.detect_shaft_client.wait_for_result(rospy.Duration(3.0))):
            self.detect_shaft_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to shaft detection returned no result. Is o2ac_vision running?")
            return False
        res = self.detect_shaft_client.get_result()
        return res
    
    def localize_object(self, object_type):
        """
        Returns object pose if object was detected in current camera view,
        False otherwise.

        item_type is the name of the mesh file in o2ac_parts_description.
        """
        self.localization_client.send_goal(o2ac_msgs.msg.localizeObjectGoal(item_id=object_type))
        if (not self.localization_client.wait_for_result(rospy.Duration(15.0))):
            self.localization_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Localization returned no result for object type " + object_type)
            return False
        
        success = False
        try:
            res = self.localization_client.get_result()
            if res.succeeded:
                return res.detected_poses[0]
        except:
            pass
        rospy.logerr("Localization failed to find a pose for object type " + object_type)
        return False