#!/usr/bin/env python

import rospy
import actionlib

from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceResult
from o2ac_routines.common import O2ACCommon

class SkillServer:
  def __init__(self):
    self.controller = O2ACCommon()
    
    self.playback_action_server = actionlib.SimpleActionServer('playback_sequence', PlayBackSequenceAction, self.execute_playback, False)
    self.playback_action_server.start()

  def execute_playback(self, goal):
    success = self.controller.playback_sequence(goal.sequence_name)
    rospy.loginfo("Playback sequence completed successfully!")
    self.playback_action_server.set_succeeded(result=PlayBackSequenceResult(success))


if __name__ == '__main__':
  rospy.init_node('skill_server_py')
  server = SkillServer()
  rospy.spin()
