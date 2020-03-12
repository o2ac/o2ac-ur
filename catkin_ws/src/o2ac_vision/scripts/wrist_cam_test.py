#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, OMRON SINIC X Corp.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import rospy
import geometry_msgs.msg
import tf_conversions
from math import pi

from o2ac_routines.base import O2ACCommonBase

class CalibrationTestClass(O2ACCommonBase):
  # Use this class to move a robot to the position of a marker it is seeing, to check a camera's calibration.
  # The handeye calibration plugin in Rviz must be running for this to work.
  # This class inherits from O2ACCommonBase, which is defined in o2ac_routines/src/o2ac_routines/base.py,
  # where convenience functions like "get_current_pose" and "go_to_pose_goal" are defined.

  def __init__(self):
    super(CalibrationTestClass, self).__init__()
    rospy.sleep(.5)

  def move_to_marker(self, robot="b_bot"):
    # We want to move the robot to a marker, but we don't want it to hit the marker.
    # We want it to move above the marker, so we apply a vertical offset of 1 cm
    # Also, we want to ignore the orientation of the marker.
    # The easiest way is to apply the orientation and offset in a fixed frame, so:

    # First we create a Pose that is on the marker
    marker_pose = geometry_msgs.msg.PoseStamped()   # "Stamped" means that a "header" is included
    marker_pose.header.frame_id = "handeye_target"  # The frame in which the pose is defined
    marker_pose.pose.orientation.w = 1.0    # This sets the orientation of the quaternion to neutral

    # Then we transform it to a fixed frame in the world, so we can apply the offset and orientation
    # The result of the next line is PoseStamped with header.frame_id = "workspace_center"
    marker_in_world = self.listener.transformPose("workspace_center", marker_pose)

    # Next, we apply a known orientation to the target pose and the vertical offset
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*3/4, pi/2))
    marker_in_world.pose.orientation = downward_orientation
    marker_in_world.pose.position.z += 0.01  # 1 cm (all ROS values are in basic SI units!)
    
    # Finally, we move the robot to the marker.
    # "end_effector_link" defines which part of the robot is moved to the target pose. It can be another frame,
    # such as a tool or the camera frame. 
    # move_lin defines if the robot will attempt a linear motion or "free motion" planning.
    self.go_to_pose_goal(robot, marker_in_world, speed=0.03, acceleration=.1, 
          end_effector_link=robot+"_robotiq_85_tip_link", move_lin=True)


if __name__ == '__main__':
  try:
    c = CalibrationTestClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to move a_bot to the marker.")
      rospy.loginfo("Enter 2 to move b_bot to the marker.")
      rospy.loginfo("Enter 3 to move to home position.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        c.move_to_marker("a_bot")
      if i == '2':
        c.move_to_marker("b_bot")
      if i == '3':
        c.go_to_named_pose("home","a_bot")
        c.go_to_named_pose("home","b_bot")
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, OMRON SINIC X Corp.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import rospy
import geometry_msgs.msg
import tf_conversions
from math import pi

from o2ac_vision.base import O2ACCommonBase

class CalibrationTestClass(O2ACCommonBase):
  # Use this class to move a robot to the position of a marker it is seeing.
  # The handeye calibration plugin in Rviz must be running for this to work.

  def __init__(self):
    super(CalibrationTestClass, self).__init__()
    rospy.sleep(.5)

  def move_to_marker(self, robot="b_bot"):
    # Create a pose in neutral orientation in the frame of the marker
    marker_pose = geometry_msgs.msg.PoseStamped()
    marker_pose.header.frame_id = "handeye_target"
    marker_pose.pose.orientation.w = 1.0

    # We do not want to hit the marker with the robot - we want to move above it.
    # We also do not want to use the orientation of the marker - we only want the position.
    # Solution: We transform the marker's pose into a world 

    # Transform the marker pose to the workspace center coordinate system
    marker_in_world = self.listener.transformPose("workspace_center", marker_pose)

    # Overwrite the orientation so we know that the robot points downwards
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*3/4, pi/2))
    marker_in_world.pose.orientation = downward_orientation

    # Finally, apply a vertical offset of 1 cm and go to the marker
    marker_in_world.pose.position.z += 0.01 
    self.go_to_pose_goal(robot, marker_in_world, speed=0.03, acceleration=.1, end_effector_link=robot+"_robotiq_85_tip_link", move_lin=True)


if __name__ == '__main__':
  try:
    c = CalibrationTestClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to move a_bot to the marker.")
      rospy.loginfo("Enter 2 to move b_bot to the marker.")
      rospy.loginfo("Enter 3 to move to home position.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        c.move_to_marker("a_bot")
      if i == '2':
        c.move_to_marker("b_bot")
      if i == '3':
        c.go_to_named_pose("home","a_bot")
        c.go_to_named_pose("home","b_bot")
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
