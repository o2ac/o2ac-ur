import actionlib
import rospy
import robotiq_msgs.msg

from ur_control.controllers import GripperController  # Simulation only


class RobotiqGripper():
    def __init__(self, namespace, use_real_robot, gripper_group):
        self.use_real_robot = use_real_robot
        self.ns = namespace

        self.opening_width = 0.0

        # Gripper
        if self.use_real_robot:
            self.sub_gripper_status_ = rospy.Subscriber("/%s/gripper_status" % self.ns, robotiq_msgs.msg.CModelCommandFeedback, self._gripper_status_callback)
            self.gripper = actionlib.SimpleActionClient('/%s/gripper_action_controller' % self.ns, robotiq_msgs.msg.CModelCommandAction)
        else:
            try:
                self.gripper = GripperController(namespace=self.ns, prefix=self.ns + '_', timeout=2.0)
            except Exception as e:
                rospy.logwarn("Fail to instantiate GripperController for simulation, " + str(e))
                rospy.logwarn("Instantiating dummy gripper, hoping for moveit Fake controllers")
                class GripperDummy():
                    pass
                self.gripper = GripperDummy()
                def close():
                    gripper_group.set_named_target("close")
                    gripper_group.go()
                def open():
                    gripper_group.set_named_target("open")
                    gripper_group.go()
                def command(cmd):
                    if cmd == "open":
                        open()
                    elif cmd == "close":
                        close()
                    else:
                        rospy.loginfo("Not supported") # TODO?
                        return
                setattr(GripperDummy, "close", lambda *args, **kwargs: close())
                setattr(GripperDummy, "open", lambda *args, **kwargs: open())
                setattr(GripperDummy, "command", lambda self, cmd: command(cmd))

    def _gripper_status_callback(self, msg):
        self.opening_width = msg.position  # [m]

    def close(self, force=40.0, velocity=.1, wait=True):
        if self.use_real_robot:
            return self.send_command("close", force=force, velocity=velocity, wait=wait)
        else:
            self.gripper.close()

    def open(self, velocity=.1, wait=True, opening_width=None):
        if self.use_real_robot:
            command = opening_width if opening_width else "open"
            return self.send_command(command, wait=wait, velocity=velocity)
        else:
            self.gripper.open()

    def send_command(self, command, this_action_grasps_an_object=False, force=40.0, velocity=.1, wait=True):
        """
        gripper: a_bot or b_bot
        command: "open", "close" or opening width
        force: Gripper force in N. From 40 to 100
        velocity: Gripper speed. From 0.013 to 0.1

        Use a slow closing speed when using a low gripper force, or the force might be unexpectedly high.
        """
        if self.use_real_robot:
            goal = robotiq_msgs.msg.CModelCommandGoal()
            goal.velocity = velocity
            goal.force = force
            if command == "close":
                goal.position = 0.0
            elif command == "open":
                goal.position = 0.140
            else:
                goal.position = command     # This sets the opening width directly
                rospy.loginfo(command)

            self.gripper.send_goal(goal)
            rospy.loginfo("Sending command " + str(command) + " to gripper: " + self.ns)
            if wait:
                self.gripper.wait_for_result(rospy.Duration(6.0))  # Default wait time: 6 s
                result = self.gripper.get_result()
                if result:
                    return True
                else:
                    return False
            else:
                return True
        else:
            self.gripper.command(command)
