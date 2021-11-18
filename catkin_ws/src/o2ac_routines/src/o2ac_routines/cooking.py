from o2ac_routines.base import *
from o2ac_routines.common import O2ACCommon

class Cooking(O2ACCommon):
    def __init__(self):
        super(Cooking, self).__init__()
        self.assembly_database.change_assembly("cooking")

    def equip_knife(self):
        self.spawn_tool("knife")
        self.planning_scene_interface.allow_collisions("knife", "")
        self.planning_scene_interface.allow_collisions("cooking_board", "")
        self.planning_scene_interface.allow_collisions("bowl", "")
        self.planning_scene_interface.allow_collisions("plate", "")

        ps_approach = geometry_msgs.msg.PoseStamped()
        ps_approach.header.frame_id = "knife_pickup_link"
        ps_approach.pose.position.x = -.04
        ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/3, 0))

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

    def load_objects(self):
        poses = [
            [0.15, 0, 0.02, 0, 0, 0],
            [0.15, 0.1, 0.02, 0, 0, 0]
        ]
        self.spawn_multiple_objects("cooking", ["cucumber", "tomato"], poses, "workspace_center")

    def pick_tomato(self):
        grasp_pose = self.get_transformed_grasp_pose("tomato", "grasp_2", target_frame="workspace_center")

        grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

        self.simple_pick("b_bot", grasp_pose, item_id_to_attach="tomato", 
                         axis="z", sign=+1, attach_with_collisions=True,
                         approach_height=0.1)

        place_pose = conversions.to_pose_stamped("cooking_board_surface", [0,0,0.02,0,tau/4,0])
        # place_pose = copy.deepcopy(grasp_pose)
        # place_pose.pose.position.x += 0.1

        self.simple_place("b_bot", place_pose, axis="z", sign=+1, place_height=0.01)

    def hold_cucumber(self):
        grasp_pose = self.get_transformed_grasp_pose("cucumber", "grasp_tip2", target_frame="workspace_center")

        ps = conversions.from_pose_to_list(grasp_pose.pose)
        print(transformations.euler_from_quaternion(ps[3:]))
        grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-3.142, 0.825, -2.093))


        self.simple_pick("b_bot", grasp_pose, item_id_to_attach="cucumber", 
                         axis="z", sign=+1, attach_with_collisions=True,
                         approach_height=0.1, lift_up_after_pick=False)
