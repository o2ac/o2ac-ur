#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import PickUpAction, PickUpGoal


class PickUpActionState(EventState):
    '''
    Actionlib for picking up an object from the tray. 
    This looks for an object with the vision system and then attempt to pick it up.

    -- object_name        string  Name of the object to be pickup

    #> gripper_opening    float   Gripper opening after pickup attempt

    <= success              PickUp sequence completed successfully.
    <= error                PickUp sequence failed to execute.

    '''

    def __init__(self, object_name):
        super(PickUpActionState, self).__init__(outcomes=['success', 'error'],
                                                output_keys=["gripper_opening"])

        self._topic = 'pickup_object'
        self._client = ProxyActionClient({self._topic: PickUpAction})  # pass required clients as dict (topic: type)
        self._object_name = object_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))
            userdata.gripper_opening = result.gripper_opening

            if not result.success:
                Logger.logwarn('Fail to complete PickUp sequence')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed PickUp sequence')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = PickUpGoal()
        goal.object_name = self._object_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the PickUp command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
