#!/usr/bin/env python

from o2ac_routines.base import O2ACBase
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from o2ac_routines.helpers import get_target_force
import numpy as np
import sys, signal
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def main():
    controller = O2ACBase()
    # controller.activate_ros_control_on_ur('b_bot')
    controller.playback_sequence("bearing_orient_totb")

    
    plane = "YZ"
    radius = 0.002
    radius_direction = "+Z"
    revolutions = 5

    steps = 100
    duration = 30.0
    
    target_force = get_target_force('-X', 5.0)
    selection_matrix = [0., 0.8, 0.8, 0.8, 0.8, 0.8]

    termination_criteria = lambda cpose: cpose[0] > 0.10727

    rospy.logwarn("** STARTING FORCE CONTROL **")
    result = controller.b_bot_compliant_arm.execute_spiral_trajectory(plane, radius, radius_direction, steps, revolutions, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(4.0), wiggle_revolutions=10.0,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.logwarn("** FORCE CONTROL COMPLETE **")

    if result != TERMINATION_CRITERIA:
        rospy.logerr("** Insertion Failed!! **")
        return

    controller.open_gripper('b_bot', wait=True)

    controller.b_bot_compliant_arm.move_relative(delta=[-0.014, 0., 0., 0., 0., 0.], wait=True, t=1.)

    pre_push_position = controller.b_bot_compliant_arm.joint_angles()

    controller.close_gripper('b_bot', velocity=0.01, wait=True)

    termination_criteria = lambda cpose: cpose[0] > 0.10727
    radius = 0.001

    rospy.logwarn("** STARTING FORCE CONTROL 2**")
    controller.b_bot_compliant_arm.execute_spiral_trajectory(plane, radius, radius_direction, steps, revolutions, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(4.0), wiggle_revolutions=10.0,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.logwarn("** FORCE CONTROL COMPLETE 2**")
    
    controller.open_gripper('b_bot', wait=True)

    rospy.logwarn("** CHANGE POSITIONS USING MOVEIT **")
    post_insertion_pose = [1.6088, -1.1894, 1.7653, -2.0387, -2.7843, -1.4562]
    controller.move_joints('b_bot', post_insertion_pose)

if __name__ == "__main__":
    main()
