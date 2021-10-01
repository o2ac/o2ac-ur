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
from aist_handeye_calibration import HandEyeCalibrationBaseRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(HandEyeCalibrationBaseRoutines):
    refposes = {
        'a_bot': (0.00, 0.00, 0.3, 0, 90,  90),
        'b_bot': (0.00, 0.00, 0.3, 0, 90, -90)
    }

    def __init__(self):
        super(InteractiveRoutines, self).__init__()
        self._effector_frame = rospy.get_param('~robot_effector_tip_frame', '')

    def run(self):
        self.go_to_named_pose('home')  # Reset pose
        axis = 'Y'

        while not rospy.is_shutdown():
            print('  r        : specify robot name to be driven')
            print('  X|Y|Z    : set translational motion axis')
            print('  R|P|W    : set rotational motion axis')
            print('  +|-      : jog motion along the current axis')
            print('  <numeric>: go to the position with the specified axis coordinate value')
            print('  o        : go to reference position')
            print('  h        : go to home position')
            print('  b        : go to back position')
            print('  q        : go to home position and quit')

            current_pose = self.get_current_pose_stamped()
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(current_pose))
            key = raw_input(prompt)

            if key == 'r':
                self._robot_name = raw_input('  robot name? ')
            elif key == 'X':
                axis = 'X'
            elif key == 'Y':
                axis = 'Y'
            elif key == 'Z':
                axis = 'Z'
            elif key == 'R':
                axis = 'Roll'
            elif key == 'P':
                axis = 'Pitch'
            elif key == 'W':
                axis = 'Yaw'
            elif key == '+':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] += 0.01
                elif axis == 'Y':
                    goal_pose[1] += 0.01
                elif axis == 'Z':
                    goal_pose[2] += 0.01
                elif axis == 'Roll':
                    goal_pose[3] += 10
                elif axis == 'Pitch':
                    goal_pose[4] += 10
                else:
                    goal_pose[5] += 10
                self.move(goal_pose)
            elif key == '-':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] -= 0.01
                elif axis == 'Y':
                    goal_pose[1] -= 0.01
                elif axis == 'Z':
                    goal_pose[2] -= 0.01
                elif axis == 'Roll':
                    goal_pose[3] -= 10
                elif axis == 'Pitch':
                    goal_pose[4] -= 10
                else:
                    goal_pose[5] -= 10
                self.move(goal_pose)
            elif is_num(key):
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] = float(key)
                elif axis == 'Y':
                    goal_pose[1] = float(key)
                elif axis == 'Z':
                    goal_pose[2] = float(key)
                elif axis == 'Roll':
                    goal_pose[3] = float(key)
                elif axis == 'Pitch':
                    goal_pose[4] = float(key)
                else:
                    goal_pose[5] = float(key)
                self.move(goal_pose)
            elif key == 'o':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'h':
                self.go_to_named_pose('home')
            elif key == 'b':
                self.go_to_named_pose('back')
            elif key == 'q':
                break

        self.go_to_named_pose('home')  # Reset pose


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    routines = InteractiveRoutines()
    routines.run()
