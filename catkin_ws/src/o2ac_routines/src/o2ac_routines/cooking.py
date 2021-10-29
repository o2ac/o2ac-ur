from o2ac_routines.base import *

class Cooking(O2ACBase):
    def __init__(self):
        super(Cooking, self).__init__()

    def equip_knife(self):
        self.spawn_tool("knife")
        self.planning_scene_interface.allow_collisions("knife", "")
        self.planning_scene_interface.allow_collisions("cooking_board", "")
        self.planning_scene_interface.allow_collisions("bowl", "")
        self.planning_scene_interface.allow_collisions("plate", "")

        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "knife_pickup_link"
        ps_approach.pose.position.x = -.04
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))

        # Define pickup pose
        ps_in_holder = copy.deepcopy(ps_approach)
        ps_in_holder.pose.position.x = .017

        self.a_bot.gripper.open()

        if not self.a_bot.go_to_named_pose("tool_pick_ready"):
            rospy.logerr("Could not move to tool_pick_ready")
            return False

        self.a_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)

        self.a_bot.go_to_pose_goal(ps_in_holder, speed=.1, move_lin=True)
        self.allow_collisions_with_robot_hand("knife", "a_bot")
        self.a_bot.gripper.close()
        self.a_bot.gripper.attach_object("knife", with_collisions=True)

        self.a_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)
            
        self.a_bot.go_to_named_pose("tool_pick_ready")
        return True

    def unequip_knife(self):
        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "knife_pickup_link"
        ps_approach.pose.position.x = -.04
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))

        # Define pickup pose
        ps_in_holder = copy.deepcopy(ps_approach)
        ps_in_holder.pose.position.x = .017

        self.a_bot.go_to_named_pose("tool_pick_ready")

        self.a_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)

        self.a_bot.go_to_pose_goal(ps_in_holder, speed=.1, move_lin=True)
        self.a_bot.gripper.open()
        self.a_bot.gripper.detach_object("knife")
        self.a_bot.gripper.forget_attached_item()
        self.planning_scene_interface.remove_attached_object(name="knife")
        self.planning_scene_interface.remove_world_object(name="knife")

        self.a_bot.go_to_pose_goal(ps_approach, speed=.1, move_lin=True)
            
        self.a_bot.go_to_named_pose("tool_pick_ready")

        self.allow_collisions_with_robot_hand("knife", "a_bot", allow=False)
        return True