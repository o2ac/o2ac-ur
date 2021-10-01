#!/usr/bin/env python

import rospy
import moveit_msgs.msg
import std_msgs.msg

import os
import rospkg


def main():
    rospy.init_node('urscript_sending_example_node', anonymous=True)
    pub = rospy.Publisher(
        "/b_bot_controller/ur_hardware_interface/script_command",
        std_msgs.msg.String,
        queue_size=1)
    rospy.sleep(1.0)  # Wait for the publisher to be registered properly

    # Construct a program to send to the robot.
    complete_program = ""

    # Read in a file
    rospack = rospkg.RosPack()
    program_template = open(
        os.path.join(
            rospack.get_path("o2ac_examples"),
            "scripts/ur",
            "move_back_forth_5cm.script"),
        'rb')
    program_line = program_template.read(1024)
    while program_line:
        complete_program += program_line
        program_line = program_template.read(1024)

    # # And/or add lines manually
    # complete_program += "def move_forward_backward_5cm(): \n"
    # complete_program += "  start_pos=get_forward_kin() \n"
    # complete_program += "  target_pos=pose_trans(start_pos,p[0.0, 0.0, 0.05, 0.0, 0.0, 0.0]) \n"
    # complete_program += "  movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), a=0.5, v=0.1) \n"
    # complete_program += "  movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], start_pos), a=0.5, v=0.1) \n"
    # complete_program += "end \n"

    # Wrap as std_msgs/String
    program_msg = std_msgs.msg.String()
    program_msg.data = complete_program

    rospy.loginfo("Sending command.")
    pub.publish(program_msg)
    rospy.loginfo("Done.")
    rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        exit(1)
    except KeyboardInterrupt:
        exit(0)
