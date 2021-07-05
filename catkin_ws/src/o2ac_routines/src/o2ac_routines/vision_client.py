import actionlib
import rospy
from aist_camera_multiplexer import RealSenseMultiplexerClient
import o2ac_msgs.msg
import std_srvs.srv
import std_msgs.msg

from o2ac_routines.helpers import check_for_real_robot


class VisionClient():
    def __init__(self):
        self.use_real_robot = rospy.get_param("use_real_robot", False)

        if self.use_real_robot:
            try:
                self.vision_multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
            except:
                self.vision_multiplexer = []
            self.multiplexer_camera_names = rospy.get_param('/camera_multiplexer/camera_names')
            self.camera_enable_services = {}
            for cam in self.multiplexer_camera_names:
                rospy.wait_for_service('/%s/enable' % cam)
                self.camera_enable_services.update({cam: rospy.ServiceProxy('/%s/enable' % cam, std_srvs.srv.SetBool)})

        self.ssd_client = actionlib.SimpleActionClient('/o2ac_vision_server/get_3d_poses_from_ssd', o2ac_msgs.msg.get3DPosesFromSSDAction)
        self.detect_shaft_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_shaft_notch', o2ac_msgs.msg.shaftNotchDetectionAction)
        self.detect_angle_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_angle', o2ac_msgs.msg.detectAngleAction)
        self.pick_success_client = actionlib.SimpleActionClient('/o2ac_vision_server/check_pick_success', o2ac_msgs.msg.checkPickSuccessAction)
        self.localization_client = actionlib.SimpleActionClient('/o2ac_vision_server/localize_object', o2ac_msgs.msg.localizeObjectAction)
        self.pulley_screw_detection_stream_client = rospy.ServiceProxy('/o2ac_vision_server/activate_pulley_screw_detection', std_srvs.srv.SetBool)
        self.pulley_screw_detection_streaming = False

    def set_camera_state(self, camera_name, enable=True):
        enable_srv = self.camera_enable_services.get(camera_name)
        rospy.loginfo("Enable => %s for camera %s" % (enable, camera_name))
        enable_srv(std_srvs.srv.SetBoolRequest(data=(enable)))
        rospy.set_param("/o2ac_vision_server/%s" % camera_name, enable)

    @check_for_real_robot
    def activate_camera(self, camera_name="b_bot_outside_camera"):
        try:
            if self.vision_multiplexer:
                # TODO(cambel): move this logic inside the multiplexer
                for cam in self.multiplexer_camera_names:
                    camera_state = rospy.get_param("/o2ac_vision_server/%s" % cam, None)
                    if camera_state is None:  # we are not sure of the state of the camera
                        try:
                            # Enable desired camera and Disable other cameras
                            self.set_camera_state(cam, cam == camera_state)
                        except:
                            pass
                    else:
                        if camera_state and cam != camera_name:
                            self.set_camera_state(cam, False)
                        elif not camera_state and cam == camera_name:
                            self.set_camera_state(cam, True)
                rospy.sleep(1)
                return self.vision_multiplexer.activate_camera(camera_name)
            else:
                rospy.logwarn("Camera multiplexer not functional! Returning true")
                return True
        except:
            pass
        rospy.logwarn("Could not activate camera! Returning false")
        return False

    @check_for_real_robot
    def read_from_ssd(self):
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

    @check_for_real_robot
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

    @check_for_real_robot
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

    def activate_pulley_screw_detection(self, activate=True):
        # Activate screw detection stream
        req = std_srvs.srv.SetBoolRequest()
        req.data = activate
        self.pulley_screw_detection_stream_client.call(req)
        self.pulley_screw_detection_streaming = activate

    def check_if_pulley_screws_visible(self):
        """ The pulley_screw_detection_stream_client needs to be set to True before calling this. """
        msg = rospy.wait_for_message('/o2ac_vision_server/pulley_screws_in_view', std_msgs.msg.Bool, rospy.Duration(1.0))
        return msg.data

    def check_pick_success(self, object_name):
        """ Returns true if the visual pick success check for the object returns True.
            This can only be used in specific pre-determined situations and for certain items.
            See the evaluatePickSuccess.action file.
        """
        goal = o2ac_msgs.msg.checkPickSuccessGoal()
        goal.item_id = object_name
        self.pick_success_client.send_goal(goal)
        if (not self.pick_success_client.wait_for_result(rospy.Duration(3.0))):
            self.pick_success_client.cancel_goal()  # Cancel goal if timeout expired
            rospy.logerr("Call to pick success returned no result. Is o2ac_vision running?")
            return False
        res = self.pick_success_client.get_result()
        return res.item_is_picked

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
