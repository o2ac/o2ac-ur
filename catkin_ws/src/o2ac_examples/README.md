# Introduction

This node contains:

1) A launch file to connect the physical robots to the MoveIt planner
2) Examples of how to use MoveIt and other things

# How to start up the physical robots

    ```
    roslaunch o2ac_examples connect_real_robots.launch
    roslaunch o2ac_moveit_config o2ac_planning_execution.launch
    ```

Don't forget to use the correct network and set the robot IPs correctly. The IPs are:
    
    a_bot (UR5, left): 192.168.1.41  
    b_bot (UR5, right): 192.168.1.42  

# How to use the examples in simulation

Start up Gazebo and MoveIt as described in the Readme of o2ac_moveit_config. Then run the example launch files of this package.

The examples are mostly in C++ because the MoveIt interface in Python is not completely up to date.


# FAQ

### How to get the correct orientation for the end effector?

One way to do it:  

- Move to the desired pose in Rviz
- Under Planning Scene/Scene Robot/Links, look up a_bot_gripper_tip_link (or the link you need)
- Enter the orientation into https://quaternions.online , obtain the Euler Angles in ZYX-order
- Enter them into this command to make it pretty: `tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/4, M_PI);`
- In Python: `pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/4, pi))`
