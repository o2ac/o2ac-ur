#!/usr/bin/env python

import roslib
import rospy
# roslib.load_manifest('ur_program_relay')

import moveit_msgs.msg
import std_msgs.msg
import o2ac_msgs.srv
import tf

import os, sys, rospkg


# Node example class.
class URScriptRelay():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.listener = tf.TransformListener()
        self.publishers = {
            'a_bot':rospy.Publisher("/a_bot/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1), 
            'b_bot':rospy.Publisher("/b_bot/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)}
        rospy.sleep(0.5) # Wait for the publishers to be registered and the listener to receive 
        
        self.rospack = rospkg.RosPack()
        self.read_templates()
        s = rospy.Service('o2ac_skills/sendScriptToUR', o2ac_msgs.srv.sendScriptToUR, self.srv_callback)

        # Main while loop.
        while not rospy.is_shutdown():
            rospy.sleep(.1)

    # Create a callback function for the service.
    def srv_callback(self, req):
        # Interpret the service parameters, construct the program, send it to the UR
        if not req.robot_name:
            rospy.logerr("robot_name was not defined in the service call to sendScriptToUR!")
            return False
        elif not req.program_id:
            rospy.logerr("No program ID was defined!")
            return False

        program_back = ""
        if req.program_id == "horizontal_insertion":
            program_front = self.horizontal_insertion_template

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Y-"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.max_radius:
                req.max_radius = .007            # in m
            if not req.radius_increment:
                req.radius_increment = 0.0003    # in m
            # if not req.peck_mode:
            #     req.peck_mode = True
            if not req.max_insertion_distance:
                req.max_insertion_distance = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    max_insertion_distance = " + str(req.max_insertion_distance) + "\n"
            program_back += "    textmsg(\"Spiral searching.\")\n"
            program_back += "    sleep(3.0)\n"
            program_back += "    if rq_spiral_search_new(max_insertion_distance," + str(req.max_force) \
                                + ", " + str(req.max_radius*1000) \
                                + ", " + str(req.radius_increment*1000) \
                                + ", peck_mode=" + str(req.peck_mode) + "):\n"
            program_back += "        #Insert the Part into the bore#\n"
            program_back += "        textmsg(\"Impedance insert\")\n"
            program_back += "        sleep(3.0)\n" 
            program_back += "        rq_impedance(max_insertion_distance, " + str(req.impedance_mass) + ")\n"
            program_back += "    end\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            

            program = program_front + "\n" + program_back

        # This activates the UR force_mode while executing a desired twist. currently the adaptive calculation is deactivated for the force_mode
        # You only need to send a goal_force, desired_twist, use_relative_pos (switches between TCP and base coordinates) and compliant_axis
        elif req.program_id == "force_mode_move":
            program_front = self.insertion_template_adaptive

            # Assign defaults
            if not req.goal_force:
                req.goal_force = [0, 0, 0, 0, 0, 0]
            if not req.goal_force:
                req.goal_force = [0, 0, 0, 0, 0, 0]
            if not req.goal_pose:
                req.goal_pose = [0, 0, 0, 0, 0, 0]
            if not req.goal_speed:
                req.goal_speed = [0, 0, 0, 0, 0, 0]
            if not req.max_force:
                req.max_force = 10.0
            if not req.compliant_axis:
                req.compliant_axis = "X"
            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Starting Adaptive Insert/force_mode_move.\")\n"
            program_back += "    force_mode_move([" + str(req.goal_force[0]) + ", " \
                                    + str(req.goal_force[1]) + ", " + str(req.goal_force[2]) + ", "\
                                    + str(req.goal_force[3]) + ", " + str(req.goal_force[4]) + ", "\
                                    + str(req.goal_force[5]) + "], [" + str(req.desired_twist[0]) + ", "\
                                    + str(req.desired_twist[1]) + ", " + str(req.desired_twist[2]) + ", "\
                                    + str(req.desired_twist[3]) + ", " + str(req.desired_twist[4]) + ", "\
                                    + str(req.desired_twist[5]) + "], [" + str(req.goal_pose[0]) + ", "\
                                    + str(req.goal_pose[1]) + ", " + str(req.goal_pose[2]) + ", "\
                                    + str(req.goal_pose[3]) + ", " + str(req.goal_pose[4]) + ", "\
                                    + str(req.goal_pose[5]) +"], [" + str(req.goal_speed[0]) + ", "\
                                    + str(req.goal_speed[1]) + ", " + str(req.goal_speed[2]) + ", "\
                                    + str(req.goal_speed[3]) + ", " + str(req.goal_speed[4]) + ", "\
                                    + str(req.goal_speed[5]) + "], " + str(req.max_force) + ", "\
                                    + str(req.use_relative_pos) + ", \"" + req.compliant_axis + "\")\n"
            program_back += "end\n"

            print(program_back)

            program = program_front + "\n" + program_back
        
        # Shifts the TCP center by req.tcp_pose and executes a movel. 
        # The TCP is reset back to default. desired_twist is a relative pose.
        elif req.program_id == "tcp_movement" or req.program_id == "tcp_move":
            program_front = self.insertion_template_adaptive
            
            program_back += "    textmsg(\"Starting TCP Movement.\")\n"
            program_back += "    tcp_movel([" + str(req.desired_twist[0]) + ", "\
                                    + str(req.desired_twist[1]) + ", " + str(req.desired_twist[2]) + ", "\
                                    + str(req.desired_twist[3]) + ", " + str(req.desired_twist[4]) + ", "\
                                    + str(req.desired_twist[5]) + "], [" + str(req.tcp_pose[0]) + ", "\
                                    + str(req.tcp_pose[1]) + ", " + str(req.tcp_pose[2]) + ", "\
                                    + str(req.tcp_pose[3]) + ", " + str(req.tcp_pose[4]) + ", " + str(req.tcp_pose[5]) + "], "\
                                    + str(req.velocity) + ", " + str(req.acceleration) +" )\n"
            program_back += "end\n"

            print(program_back)

            program = program_front + "\n" + program_back
        
        # Shifts the TCP center with req.tcp_pose
        elif req.program_id == "set_tcp":
            program_front = self.insertion_template_adaptive

            # Assign defaults
            if not req.tcp_pose:
                req.tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            program_back += "    textmsg(\"Changing TCP.\")\n"
            program_back += "    set_tcp(p[" + str(req.tcp_pose[0]) + ", " + str(req.tcp_pose[1]) + ", "\
                                            + str(req.tcp_pose[2]) + ", " + str(req.tcp_pose[3]) + ", "\
                                            + str(req.tcp_pose[4]) + ", " + str(req.tcp_pose[5]) + "])\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
            
        elif req.program_id == "current_robot_speed":
            program_front = self.insertion_template

            program_back += "    textmsg(\"Current robot pose:\", get_actual_tcp_speed())\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back

        # Performs a spiral motion in a plane
        elif req.program_id == "spiral":
            program_front = self.insertion_template

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.max_radius:
                req.max_radius = .004            # in m
            if not req.radius_increment:
                req.radius_increment = 0.0003    # in m
            if not req.peck_mode:
                req.peck_mode = False
            if not req.max_insertion_distance:
                req.max_insertion_distance = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    max_insertion_distance = " + str(req.max_insertion_distance) + "\n"
            program_back += "    textmsg(\"Spiral searching.\")\n"
            program_back += "    sleep(1.0)\n"
            program_back += "    rq_spiral_search_new(max_insertion_distance," + str(req.max_force) \
                                + ", " + str(req.max_radius*1000) \
                                + ", " + str(req.radius_increment*1000) \
                                + ", peck_mode=" + str(req.peck_mode) + ")\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back

        elif req.program_id == "test_force_mode":
            program_front = self.insertion_template_adaptive

            # Test for force_mode movement. The force value increases every 5 seconds and is applied to the UR force_mode in x-direction
            # X is also the compliant axis. Once it runs through all force-array values in f, it starts from the beginning.
            program_back += "    textmsg(\"Starting Force Mode.\")\n"
            program_back += "    f = [0,0]\n"
            program_back += "    counter = 0\n"
            program_back += "    while True:\n"
            program_back += "        force_mode( get_actual_tcp_pose() ," \
                                    + "[1,0,0,0,0,0]" + "," \
                                    + "[f[counter],0,0,0,0,0]" + "," \
                                    + "2" + "," \
                                    + "[200,150,200,10,10,10]" + ")\n"
            # program_back += "        textmsg(\"external force:\", force())\n"
            # program_back += "        textmsg(\"f:\", f[counter])\n"
            program_back += "        counter = counter + 1\n"
            program_back += "        if counter > 1:\n"
            program_back += "            counter = 0\n"
            program_back += "        end\n"
            program_back += "        sleep(5)\n"
            program_back += "    end\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back  
            
        elif req.program_id == "insertion" or req.program_id == "insert" or req.program_id == "spiral_search":
            program_front = self.insertion_template

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.max_radius:
                req.max_radius = .004            # in m
            if not req.radius_increment:
                req.radius_increment = 0.0003    # in m
            if not req.peck_mode:
                req.peck_mode = False
            if not req.max_insertion_distance:
                req.max_insertion_distance = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    max_insertion_distance = " + str(req.max_insertion_distance) + "\n"
            program_back += "    textmsg(\"Spiral searching.\")\n"
            program_back += "    sleep(3.0)\n"
            program_back += "    if rq_spiral_search_new(max_insertion_distance," + str(req.max_force) \
                                + ", " + str(req.max_radius*1000) \
                                + ", " + str(req.radius_increment*1000) \
                                + ", peck_mode=" + str(req.peck_mode) + "):\n"
            program_back += "        #Insert the Part into the bore#\n"
            if req.program_id != "spiral_search":
                program_back += "        textmsg(\"Impedance insert\")\n"
                program_back += "        sleep(3.0)\n" 
                program_back += "        rq_impedance(max_insertion_distance, " + str(req.impedance_mass) + ")\n"
            else:
                program_back += "        textmsg(\"Spiral search over.\")\n"
            program_back += "    end\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        
        # Moves forward until a force is detected
        elif req.program_id == "linear_push":
            program_front = self.linear_push_template

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.acceleration:
                req.acceleration = 1.0
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.use_base_coords:
                use_base_coords = False

            if len(req.direction_vector) == 3:
                dir_vec = req.direction_vector
            if (req.force_direction == "X+") or (req.force_direction == "x+"):
                dir_vec = [1.0, 0, 0]
            elif (req.force_direction == "X-") or (req.force_direction == "x-"):
                dir_vec = [-1.0, 0, 0]
            elif (req.force_direction == "Y+") or (req.force_direction == "y+"):
                dir_vec = [0, 1.0, 0]
            elif (req.force_direction == "Y-") or (req.force_direction == "y-"):
                dir_vec = [0, -1.0, 0]
            elif (req.force_direction == "Z+") or (req.force_direction == "z+"):
                dir_vec = [0, 0, 1.0]
            elif (req.force_direction == "Z-") or (req.force_direction == "z-"):
                dir_vec = [0, 0, -1.0]
            
            ### Copied from linear_search_test.script, which is the urscript generated by the UR5e pendant for "move in direction until contact"
            program_back += "  global move_thread_flag_3=0\n"
            program_back += "  dir_vec = [" + str(dir_vec[0]) + ","+ str(dir_vec[1]) + ","+ str(dir_vec[2]) + "]\n"
            # program_back += "  dir_vec = [0.0,0.0,-1.0]\n"
            program_back += "  thread move_thread_3():\n"
            program_back += "      enter_critical\n"
            program_back += "      move_thread_flag_3 = 1\n"
            if use_base_coords: # Robot base coordinate system:
                program_back += "      local towardsPos=calculate_point_to_move_towards(p[0.0,0.0,0.0,0.0,0.0,0.0], dir_vec, 1000.0)\n"
            else: # Tool coordinate system:
                program_back += "      local towardsPos=calculate_point_to_move_towards(get_forward_kin(), dir_vec, 1000.0)\n"
            program_back += "      movel(towardsPos, a=" + str(req.acceleration) + ", v=" + str(req.forward_speed) + ")\n"
            program_back += "      move_thread_flag_3 = 2\n"
            program_back += "      exit_critical\n"
            program_back += "  end\n"
            program_back += "  move_thread_flag_3 = 0\n"
            program_back += "  move_thread_han_3 = run move_thread_3()\n"
            program_back += "  while (True):\n"
            program_back += "      local targetTcpDirection=get_target_tcp_speed()\n"
            program_back += "      local stepsToRetract=tool_contact(direction=targetTcpDirection)\n"
            program_back += "      if (stepsToRetract > 0):\n"
            program_back += "      kill move_thread_han_3\n"
            program_back += "      stopl(3.0)\n"
            program_back += "      local backTrackMovement=get_actual_joint_positions_history(stepsToRetract)\n"
            program_back += "      local contactPose=get_forward_kin(backTrackMovement)\n"
            program_back += "      local posDir=[targetTcpDirection[0],targetTcpDirection[1],targetTcpDirection[2]]\n"
            program_back += "      local retractTo=contactPose\n"
            program_back += "      if (norm(posDir) > 1e-6):\n"
            program_back += "          local normalizedPosDir=normalize(posDir)\n"
            program_back += "          local additionalRetraction=p[normalizedPosDir[0] * 0.0, normalizedPosDir[1] * 0.0, normalizedPosDir[2] * 0.0, 0, 0, 0]\n"
            program_back += "          retractTo = pose_sub(contactPose, additionalRetraction)\n"
            program_back += "      end\n"
            program_back += "      movel(retractTo, a=3.0, v=0.1)\n"
            program_back += "      break\n"
            program_back += "      end\n"
            program_back += "      sync()\n"
            program_back += "  end\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        elif req.program_id == "lin_move":
            rospy.logdebug("Note: UR script lin move uses the ee_link of the robot, not the EE of the move group.") 
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = .03
            rospy.logdebug("original pose:")
            rospy.logdebug(req.target_pose)
            transform_success = False
            counter = 50
            while not transform_success:
                try:
                    counter += 1
                    robot_pose = self.listener.transformPose(req.robot_name + "_base", req.target_pose)
                    transform_success = True
                except:
                    rospy.logdebug("Failed to transform from frame " + req.target_pose.header.frame_id + ". Waiting for .1 seconds")
                    rospy.sleep(.1)
            xyz = [robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z]
            
            q = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, 
                 robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]
            rpy = tf.transformations.euler_from_quaternion(q)
            # rpy needs to be in axis-angle representation
            # http://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
            rospy.logdebug("q in robot base:")
            rospy.logdebug(q)

            # This seems to work, but it uses the ee_link TCP of the robot.
            program = ""
            program += "def move_to_pose_lin():\n"
            program += "    textmsg(\"Move_l to a pose.\")\n"
            program += "    rv = rpy2rotvec([" + str(rpy[0]) + "," + str(rpy[1]) + "," + str(rpy[2]) + "])\n"
            program += "    target_pos=p[" + str(xyz[0]) + "," + str(xyz[1]) + "," + str(xyz[2]) + "," \
                                      "rv[0], rv[1], rv[2]]\n"
            program += "    movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), " + \
                            "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            # program += "    movel(target_pos, " + \
            #                 "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
            rospy.logdebug(program)

        elif req.program_id == "lin_move_rel" or req.program_id == "move_lin_rel":
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = .03
            # rospy.logwarn("The frame_id of the Point is ignored!")
            xyz = [req.relative_translation.x, req.relative_translation.y, req.relative_translation.z]
            rot = [req.relative_rotation.x, req.relative_rotation.y, req.relative_rotation.z]

            program = ""
            program += "def move_lin_rel():\n"
            program += "    textmsg(\"Move_l via relative translation.\")\n"
            program += "    offset_pose = p[" + str(xyz[0]) + ", " + str(xyz[1]) + ", " + str(xyz[2]) + "," \
                                    + str(rot[0]) + ", " + str(rot[1]) + ", " + str(rot[2]) + "]\n"
            program += "    current_pos = get_actual_tcp_pose()\n"
            if not req.lin_move_rel_in_base_csys: # Move relative to TCP
                program += "    movel(pose_trans(current_pos, offset_pose), " + \
                                "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            else: # Move relative to robot base
                program += "    movel(pose_trans(offset_pose, current_pos), " + \
                                "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
        elif req.program_id == "spiral_press":
            rospy.logerr("SPIRAL PRESS IS NOT IMPLEMENTED YET") # TODO: Is this not just the "insertion" script with peck_mode=False?
        elif req.program_id == "spiral_motion":
            program_front = self.spiral_motion_template
            if (req.radius_increment < 0.0001) or (req.radius_increment > 0.05):
                rospy.logwarn("radius_incr needs to be between 0.0001 and 0.05 but is " + str(req.radius_increment))
            if not req.acceleration:
                req.acceleration = 0.1
            if not req.velocity:
                req.velocity = .03
            if not req.max_radius:
                req.max_radius = .0065
            if not req.radius_increment:
                req.radius_increment = .002
            if not req.theta_increment:
                req.theta_increment = 30
            if not req.spiral_axis:
                req.spiral_axis = "Z"
            
            program_back += "    textmsg(\"Performing spiral motion.\")\n"
            program_back += "    spiral_motion(" + str(req.max_radius) \
                                + ", " + str(req.radius_increment) \
                                + ", " + str(req.velocity) \
                                + ", " + str(req.acceleration) \
                                + ", \"" + req.spiral_axis + "\"" \
                                + ", " + str(req.theta_increment) + ")\n"
            program_back += "    textmsg(\"Done.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        
        # Moves outwards in a spiral, then moves downwards while moving in a spiral
        elif req.program_id == "helix_motion":
            program_front = self.helix_motion_template
            if (req.radius_increment < 0.0001) or (req.radius_increment > 0.005):
                rospy.logerr("radius_incr needs to be between 0.0001 and 0.005 but is " + str(req.radius_increment))
            if not req.acceleration:
                req.acceleration = 0.1
            if not req.velocity:
                req.velocity = .03
            if not req.max_force:
                req.max_force = 20
            if not req.max_radius:
                req.max_radius = .005
            if not req.radius_increment:
                req.radius_increment = .004
            if not req.theta_increment:
                req.theta_increment = 30
            if not req.spiral_axis:
                req.spiral_axis = "Z"
            if not req.helix_forward_axis:
                req.helix_forward_axis = "Z"
            if not req.helix_forward_increment:
                req.helix_forward_increment = 0.01
            if not req.helix_forward_limit:
                req.helix_forward_limit = 0.05
            
            program_back += "    textmsg(\"Performing helix motion.\")\n"
            program_back += "    helix_motion(" + str(req.max_radius) \
                                + ", " + str(req.radius_increment) \
                                + ", " + str(req.velocity) \
                                + ", " + str(req.acceleration) \
                                + ", \"" + req.spiral_axis + "\"" \
                                + ", " + str(req.theta_increment) \
                                + ", \"" + req.helix_forward_axis + "\"" \
                                + ", " + str(req.helix_forward_increment) \
                                + ", " + str(req.helix_forward_limit) \
                                + ", " + str(req.max_force) + ")\n"
            program_back += "    textmsg(\"Done.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        elif req.program_id == "test":
            program = ""
            program_file = open(os.path.join(self.rospack.get_path("o2ac_examples"), "scripts/script_command", "move_back_forth_5cm.script"), 'rb')
            program_line = program_file.read(1024)
            while program_line:
                program += program_line
                program_line = program_file.read(1024)
        elif req.program_id == "move_j":

            if not len(req.joint_positions) == 6:
                rospy.logwarn("Joint pose vector not of the correct length")
                return False
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = 0.5

            program = ""
            program += "def move_to_joint_pose():\n"
            program += "    textmsg(\"Move_j to a pose.\")\n"
            program += "    target_pos=[" + str(req.joint_positions[0]) + "," + str(req.joint_positions[1]) + "," + str(req.joint_positions[2]) + "," \
                                      + str(req.joint_positions[3]) + "," + str(req.joint_positions[4]) + "," + str(req.joint_positions[5]) + "]\n"
            program += "    movej(target_pos, " + \
                            "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
            rospy.logdebug(program)
        else:
            rospy.logerr("The program could not be recognized: " + req.program_id)
            return False

        # Send the program to the robot
        program_msg = std_msgs.msg.String()
        program_msg.data = program

        rospy.loginfo("Sending UR robot program " + req.program_id)
        rospy.logdebug("Program is:")
        rospy.logdebug(program)
        self.publishers[req.robot_name].publish(program_msg)
        return True

    def read_templates(self):
        # Read the files containing the program templates into memory
        self.insertion_template = self.read_template("peginholespiral_imp_osx.script")
        self.horizontal_insertion_template = self.read_template("peginholespiral_imp_osx_y_negative.script")
        self.linear_push_template = self.read_template("linear_search_ur5e.script")
        self.spiral_motion_template = self.read_template("spiral_motion.script")
        self.helix_motion_template = self.read_template("helix_motion.script")
        self.insertion_template_adaptive = self.read_template("osx_adaptive_insertion.script")
        return True

    def read_template(self, filename):
        program_template_file = open(os.path.join(self.rospack.get_path("o2ac_skills"), "src/urscript", filename), 'rb')
        program_line = program_template_file.read(1024)
        linecounter = 0
        template = ""
        while program_line:
            template += program_line
            program_line = program_template_file.read(1024)
        return template

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('o2ac_urscript_construction_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = URScriptRelay()
    except rospy.ROSInterruptException: pass
