#! /usr/bin/env python

# This script shows how to call the vision actions.

import rospy
import argparse
import actionlib
import o2ac_msgs.msg


def get_2d_poses_from_ssd_client():
    client = actionlib.SimpleActionClient(
        "get_2d_poses_from_ssd", o2ac_msgs.msg.get2DPosesFromSSDAction
    )
    client.wait_for_server()

    goal = o2ac_msgs.msg.get2DPosesFromSSDGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def belt_detection_client():
    client = actionlib.SimpleActionClient(
        "belt_detection", o2ac_msgs.msg.beltDetectionAction
    )
    client.wait_for_server()

    goal = o2ac_msgs.msg.get2DPosesFromSSDGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":

    try:
        rospy.init_node("test_client_py")

        results = get_2d_poses_from_ssd_client()
        print "pose estimation result"
        for j in range(len(results.get_2d_poses_from_ssd_result_list)):
            print "--------------------------------"
            print "confidence: ", results.get_2d_poses_from_ssd_result_list[
                j
            ].confidence
            print "rotation [deg(ccw)]: ", results.get_2d_poses_from_ssd_result_list[
                j
            ].rotation
            print "center [j,i]: ", results.get_2d_poses_from_ssd_result_list[j].center
            # print "3d pose: ",
            # results.get_2d_poses_from_ssd_result_list[j].pose
            print "class id: ", results.get_2d_poses_from_ssd_result_list[j].class_id
            # print "grasp points: ",
            # results.get_2d_poses_from_ssd_result_list[j].grasp_points

        results = belt_detection_client()
        print "\nbelt detection result"
        print "grasp points: ", results.grasp_points

    except rospy.ROSInterruptException:
        pass
