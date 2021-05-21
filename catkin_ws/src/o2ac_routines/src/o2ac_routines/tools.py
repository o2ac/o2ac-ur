import actionlib
import o2ac_msgs.msg
import rospy
from std_msgs.msg import Bool

from o2ac_routines.helpers import check_for_real_robot

class Tools():
    def __init__(self):
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.suction_client = actionlib.SimpleActionClient('/suction_control', o2ac_msgs.msg.SuctionControlAction)
        self.fastening_tool_client = actionlib.SimpleActionClient('/screw_tool_control', o2ac_msgs.msg.ScrewToolControlAction)

        self.sub_suction_m4_ = rospy.Subscriber("/screw_tool_m4/screw_suctioned", Bool, self.suction_m4_callback)
        self.sub_suction_m3_ = rospy.Subscriber("/screw_tool_m3/screw_suctioned", Bool, self.suction_m3_callback)

        self.screw_is_suctioned = dict()


    def suction_m4_callback(self, msg):
        self.screw_is_suctioned["m4"] = msg.data

    def suction_m3_callback(self, msg):
        self.screw_is_suctioned["m3"] = msg.data

    @check_for_real_robot
    def set_suction(self, tool_name, suction_on=False, eject=False, wait=True):
        goal = o2ac_msgs.msg.SuctionControlGoal()
        goal.fastening_tool_name = tool_name
        goal.turn_suction_on = suction_on
        goal.eject_screw = eject
        rospy.loginfo("Sending suction action goal.")
        self.suction_client.send_goal(goal)
        if wait:
            self.suction_client.wait_for_result(rospy.Duration(2.0))
        return self.suction_client.get_result()

    @check_for_real_robot
    def set_motor(self, motor_name, direction="tighten", wait=False, speed=0, duration=0, skip_final_loosen_and_retighten=False):
        goal = o2ac_msgs.msg.ScrewToolControlGoal()
        goal.fastening_tool_name = motor_name
        goal.direction = direction
        goal.speed = speed
        goal.duration = duration
        goal.skip_final_loosen_and_retighten = skip_final_loosen_and_retighten
        rospy.loginfo("Sending fastening_tool action goal.")
        self.fastening_tool_client.send_goal(goal)
        if wait:
            self.fastening_tool_client.wait_for_result()
        return self.fastening_tool_client.get_result()
