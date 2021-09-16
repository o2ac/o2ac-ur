#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import MoveToAction, MoveToGoal


class MoveToActionState(EventState):
    '''
    Actionlib for aligning the bearing holes

    -- robot_name        string  Name of the object to be MoveTo
    -- target_pose        float[]
    -- pose_type          string
    -- frame_id           string
    -- target_named_pose  string
    -- motion_planner     string

    <= success              MoveTo completed successfully.
    <= error                MoveTo failed to execute.

    '''

    def __init__(
            self,
            robot_name,
            target_pose,
            pose_type="joints",
            frame_id="world",
            target_named_pose="",
            motion_planner="linear"):
        super(MoveToActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/MoveTo'
        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient({self._topic: MoveToAction})
        self._robot_name = robot_name
        self._target_pose = target_pose
        self._pose_type = pose_type
        self._frame_id = frame_id
        self._target_named_pose = target_named_pose
        self._motion_planner = motion_planner

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result.success:
                Logger.logwarn('Fail to complete MoveTo')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed MoveTo')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = MoveToGoal()
        goal.robot_name = self._robot_name
        goal.target_pose = self._target_pose
        goal.pose_type = self._pose_type
        goal.frame_id = self._frame_id
        goal.target_named_pose = self._target_named_pose
        goal.motion_planner = self._motion_planner

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the MoveTo command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
