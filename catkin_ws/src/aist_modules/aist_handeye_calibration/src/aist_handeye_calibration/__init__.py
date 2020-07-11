import rospy
import dynamic_reconfigure.client
import std_srvs.srv

from geometry_msgs      import msg as gmsg
from tf                 import transformations as tfs
from math               import radians, degrees
from o2ac_routines.base import O2ACBase

######################################################################
#  class HandEyeCalibrationBaseRoutines                              #
######################################################################
class HandEyeCalibrationBaseRoutines(O2ACBase):
    def __init__(self):
        super(HandEyeCalibrationBaseRoutines, self).__init__()

        camera_name = rospy.get_param('~camera_name', 'a_bot_inside_camera')
        camera_type = rospy.get_param('~camera_type', 'DepthCamera')
        self._camera         = DepthCamera.create(camera_name, camera_type)
        self._robot_name     = rospy.get_param('~robot_name', 'a_bot')
        self._base_frame     = rospy.get_param('~robot_base_frame',
                                               'workspace_center')
        self._effector_frame = rospy.get_param('~robot_effector_frame',
                                               self._robot_name + '_ee_link')
        self._speed          = rospy.get_param('~speed', 0.1)
        self._initpose       = rospy.get_param('~initpose', None)

    # Robot stuffs
    def get_current_pose_stamped(self):
        return super(HandEyeCalibrationBaseRoutines,
                     self).get_current_pose_stamped(self._robot_name)

    def go_to_named_pose(self, named_pose):
        return super(HandEyeCalibrationBaseRoutines,
                     self).go_to_named_pose(named_pose, self._robot_name)

    def go_to_init_pose(self, verbose=False):
        if self._initpose:
            return self.move(self._initpose, verbose)
        else:
            return False, False

    def move(self, xyzrpy, verbose=False):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self._base_frame
        target_pose.pose = gmsg.Pose(gmsg.Point(*xyzrpy[0:3]),
                                     gmsg.Quaternion(
                                         *tfs.quaternion_from_euler(
                                             *map(radians, xyzrpy[3:6]))))
        if verbose:
            print('  move to ' + self.format_pose(target_pose))
        res = self.go_to_pose_goal(self._robot_name, target_pose,
                                   end_effector_link=self._effector_frame,
                                   speed=self._speed)
        if verbose:
            print('  reached '
                  + self.format_pose(self.get_current_pose_stamped()))
        return res

    # Camera stuffs
    def continuous_shot(self, enable):
        return self._camera.continuous_shot(enable)

    def trigger_frame(self):
        return self._camera.trigger_frame()

    # Utility functions
    def transform_pose_to_reference_frame(self, pose):
        try:
            pose.header.stamp = rospy.Time.now()
            self.listener.waitForTransform(self._base_frame,
                                           pose.header.frame_id,
                                           pose.header.stamp,
                                           rospy.Duration(10))
            return self.listener.transformPose(self._base_frame, pose)
        except Exception as e:
            rospy.logerr('transform_pose_to_reference_frame(): {}'.format(e))
            raise e

    def xyz_rpy(self, pose):
        transformed_pose = self.transform_pose_to_reference_frame(pose).pose
        rpy = tfs.euler_from_quaternion([transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w])
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

    def format_pose(self, pose):
        xyzrpy = self.xyz_rpy(pose)
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            *xyzrpy)

    def effector_target_pose(self, target_pose, offset):
        T = tfs.concatenate_matrices(
                self.listener.fromTranslationRotation(
                    (target_pose.pose.position.x,
                     target_pose.pose.position.y,
                     target_pose.pose.position.z),
                    (target_pose.pose.orientation.x,
                     target_pose.pose.orientation.y,
                     target_pose.pose.orientation.z,
                     target_pose.pose.orientation.w)),
                self.listener.fromTranslationRotation(
                    offset,
                    tfs.quaternion_from_euler(0, radians(90), 0)))
        pose = gmsg.PoseStamped()
        pose.header.frame_id = target_pose.header.frame_id
        pose.pose = gmsg.Pose(gmsg.Point(*tfs.translation_from_matrix(T)),
                              gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        return pose

######################################################################
#  class DepthCamera                                                 #
######################################################################
class DepthCamera(object):
    def __init__(self, name):
        self._name = name

    @staticmethod
    def create(name, type_name):
        ClientClass = globals()[type_name]
        if rospy.get_param("use_real_robot", False):
            return ClientClass(name)
        else:
            return ClientClass.base(name)

    @staticmethod
    def base(name):
        return DepthCamera(name)

    @property
    def name(self):
        return self._name

    def trigger_frame(self):
        return True

######################################################################
#  class PhoXiCamera                                                 #
######################################################################
class PhoXiCamera(DepthCamera):
    def __init__(self, name="a_phoxi_m_camera"):
        super(PhoXiCamera, self).__init__(name)
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._trigger_frame = rospy.ServiceProxy(name + "/trigger_frame",
                                                 std_srvs.srv.Trigger)
        # Enable trigger mode
        self._dyn_reconf.update_configuration({"trigger_mode" : 1})

    def trigger_frame(self):
        return self._trigger_frame().success
