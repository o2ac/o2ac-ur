#!/usr/bin/env python

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
