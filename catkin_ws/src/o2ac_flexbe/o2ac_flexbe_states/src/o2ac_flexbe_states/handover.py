#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import HandoverAction, HandoverGoal


class HandoverActionState(EventState):
    '''
    Actionlib for executing an Handoverion for a given object and task

    -- from_robot_name         string  Name of robot performing the operation
    -- to_robot_name         string  Name of robot performing the operation
    -- object_name        string  Name object to be Handovered

    <= success              Handover sequence completed successfully.
    <= error                Handover sequence failed to execute.

    '''

    def __init__(self, from_robot_name, to_robot_name, object_name):
        super(
            HandoverActionState,
            self).__init__(
            outcomes=[
                'success',
                'error'])

        self._topic = 'o2ac_flexbe/handover'
        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient({self._topic: HandoverAction})
        self._from_robot_name = from_robot_name
        self._to_robot_name = to_robot_name
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('Handover result %s' % str(result))  # for debug

            if not result.success:
                Logger.logwarn('Fail to complete Handover')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed Handover')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = HandoverGoal()
        goal.from_robot_name = self._from_robot_name
        goal.to_robot_name = self._to_robot_name
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn(
                'Failed to send the Handoverion command:\n%s' %
                str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
