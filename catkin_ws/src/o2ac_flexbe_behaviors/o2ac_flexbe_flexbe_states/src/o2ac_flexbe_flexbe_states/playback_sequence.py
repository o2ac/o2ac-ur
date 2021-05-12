#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceGoal


class PlayBackActionState(EventState):
    '''
    Actionlib for executing a playback sequence 

    -- sequence_name        string  Name of the recorded playback sequence

    <= success              Playback sequence completed successfully.
    <= error                playback sequence failed to execute.

    '''

    def __init__(self, sequence_name):
        super(PlayBackActionState, self).__init__(outcomes=['success', 'error'])

        self._topic = 'playback_sequence'
        self._client = ProxyActionClient({self._topic: PlayBackSequenceAction})  # pass required clients as dict (topic: type)
        self._sequence_name = sequence_name

        self._success = False

    def execute(self, userdata):
        if not self._success:
            return 'error'

        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            Logger.logwarn('result %s' % str(result))

            if not result:
                Logger.logwarn('Fail to complete playback sequence')
                self._success = False
                return 'error'
            else:
                Logger.logwarn('Succeed! completed playback sequence')
                self._success = True
                return 'success'

    def on_enter(self, userdata):
        goal = PlayBackSequenceGoal()
        goal.sequence_name = self._sequence_name

        self._success = True
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the PlayBackSequence command:\n%s' % str(e))
            self._success = False

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
