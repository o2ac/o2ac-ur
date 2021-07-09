#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import PickAction, PickGoal


class PickActionState(EventState):
    '''
    Actionlib for picking up an object from the tray. 
    This looks for an object with the vision system and then attempt to pick it up.

    -- object_name        string  Name of the object to be pick

    #> gripper_opening    float   Gripper opening after pick attempt

    <= success              Pick sequence completed successfully.
    <= error                Pick sequence failed to execute.

    '''

    def __init__(self, object_name):
        super(PickActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'o2ac_flexbe/pick_object'
        self._client = ProxyActionClient({self._topic: PickAction})  # pass required clients as dict (topic: type)
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result.success:
                Logger.logwarn('Fail to complete Pick sequence')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed Pick sequence')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = PickGoal()
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Pick command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
