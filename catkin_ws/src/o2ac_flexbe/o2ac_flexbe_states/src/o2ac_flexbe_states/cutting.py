#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import CuttingAction, CuttingGoal


class CuttingActionState(EventState):
    '''
    Actionlib for Cuttinging up an object on the cutting board.
    This looks for an object with the vision system and then attempt to Cutting it up.

    -- robot_name         string  Name of robot performing the operation
    -- object_name        string  Name of the object to be Cutting

    <= success              Cutting sequence completed successfully.
    <= error                Cutting sequence failed to execute.

    '''

    def __init__(self, robot_name, object_name):
        super(CuttingActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/cutting'
        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient({self._topic: CuttingAction})
        self._robot_name = robot_name
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result.success:
                Logger.logwarn('Fail to complete Cutting sequence')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed Cutting sequence')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = CuttingGoal()
        goal.robot_name = self._robot_name
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Cutting command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
