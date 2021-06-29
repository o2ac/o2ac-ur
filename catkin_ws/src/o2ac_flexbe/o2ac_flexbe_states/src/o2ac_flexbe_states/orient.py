#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import OrientAction, OrientGoal


class OrientActionState(EventState):
    '''
    Actionlib for aligning the bearing holes

    -- task_name        string  Name of the task
    -- object_name      string  Name of the object to be oriented

    <= success              Orient completed successfully.
    <= error                Orient failed to execute.

    '''

    def __init__(self, task_name, object_name):
        super(OrientActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/orient'
        self._client = ProxyActionClient({self._topic: OrientAction})  # pass required clients as dict (topic: type)
        self._task_name = task_name
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result:
                Logger.logwarn('Fail to complete Orient')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed Orient')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = OrientGoal()
        goal.task_name = self._task_name
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Orient command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
