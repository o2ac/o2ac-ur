#! /usr/bin/env python

import rospy
import actionlib
import o2ac_msgs.msg

def M2Det_test_client():
    client = actionlib.SimpleActionClient('M2DetTest', o2ac_msgs.msg.M2DetTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.M2DetTestGoal()
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('M2Det_test_client_py')

        # Call M2DetTest.action
        result = M2Det_test_client()
        for j in range(len(result.M2Det_result_list)):
            print '================================'
            for i in range(len(result.M2Det_result_list[j].M2Det_result)):
                print result.M2Det_result_list[j].M2Det_result[i].bboxes
                print result.M2Det_result_list[j].M2Det_result[i].label
                print result.M2Det_result_list[j].M2Det_result[i].confidence

    except rospy.ROSInterruptException:
        pass
