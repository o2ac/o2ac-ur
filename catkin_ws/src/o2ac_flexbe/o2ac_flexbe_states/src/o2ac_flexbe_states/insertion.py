#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import InsertAction, InsertGoal


class InsertActionState(EventState):
    '''
    Actionlib for executing an insertion for a given object and task

    -- robot_name         string  Name of robot performing the operation
    -- task_name          string  Name the task (insertion parameters dependent on task)
    -- object_name        string  Name object to be inserted

    <= success              Insertion sequence completed successfully.
    <= error                Insertion sequence failed to execute.

    '''

    def __init__(self, robot_name, task_name, object_name):
        super(InsertActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/force_insertion'
        self._client = ProxyActionClient({self._topic: InsertAction})  # pass required clients as dict (topic: type)
        self._robot_name = robot_name
        self._task_name = task_name
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('insert result %s' % str(result))  # for debug

            if not result.success:
                Logger.logwarn('Fail to complete insertion of %s in task %s' % (self._object_name, self._task_name))
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed insertion of %s in task %s' % (self._object_name, self._task_name))
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = InsertGoal()
        goal.robot_name = self._robot_name
        goal.task_name = self._task_name
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Insertion command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
