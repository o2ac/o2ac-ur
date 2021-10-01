#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import rospy
from geometry_msgs            import msg as gmsg
from aist_handeye_calibration import HandEyeCalibrationBaseRoutines

######################################################################
#  class CheckCalibrationRoutines                                    #
######################################################################
class CheckCalibrationRoutines(HandEyeCalibrationBaseRoutines):
    def __init__(self):
        super(CheckCalibrationRoutines, self).__init__()
        self._effector_frame = rospy.get_param('~robot_effector_tip_frame', '')

    def move_to_marker(self):
        self.trigger_frame()
        marker_pose = rospy.wait_for_message('/aruco_detector/pose',
                                             gmsg.PoseStamped, 10)
        approach_pose = self.effector_target_pose(marker_pose, (0, 0, 0.05))

        #  We have to transform the target pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        target_pose = self.transform_pose_to_reference_frame(
                          self.effector_target_pose(marker_pose, (0, 0, 0)))
        self.active_robots[self._robot_name].go_to_pose_goal(approach_pose,
                             end_effector_link=self._effector_frame,
                             speed=self._speed)
        rospy.sleep(1)
        self.active_robots[self._robot_name].go_to_pose_goal(target_pose,
                             end_effector_link=self._effector_frame,
                             speed=self._speed)

    def run(self):
        self.ab_bot.go_to_named_pose('home')

        while not rospy.is_shutdown():
            try:
                print('\n  RET: go to the marker')
                print('  i  : go to initial position')
                print('  h  : go to home position')
                print('  q  : go to home position and quit')
                key = raw_input('>> ')
                if key == 'i':
                    self.go_to_init_pose()
                elif key == 'h':
                    self.ab_bot.go_to_named_pose('home')
                elif key == 'q':
                    break
                else:
                    self.move_to_marker()
            except rospy.ROSException as ex:
                rospy.logwarn(ex.message)
            except Exception as ex:
                rospy.logerr(ex)
                break

        self.ab_bot.go_to_named_pose('home')

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('check_calibration')
    routines = CheckCalibrationRoutines()
    routines.run()
