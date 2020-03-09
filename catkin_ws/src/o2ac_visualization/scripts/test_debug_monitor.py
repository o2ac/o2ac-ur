#! /usr/bin/env python
from time import sleep, time
import rospy
from std_msgs.msg import String, Int32
from o2ac_visualization.msg import PressureSensoState

if __name__ == "__main__":
  # Initialize the ROS node
  rospy.init_node("test_debug_monitor")

  # Initialize publishers
  pub1 = rospy.Publisher("/o2ac_state/kitting_set_id", Int32, queue_size=1)
  pub2 = rospy.Publisher("/o2ac_state/task", String, queue_size=1)
  pub3 = rospy.Publisher("/o2ac_state/subtask", String, queue_size=1)
  pub4 = rospy.Publisher("/o2ac_state/operation", String, queue_size=1)
  pub5 = rospy.Publisher("/o2ac_fastening_tools/screw_suctioned",
                         PressureSensoState, queue_size=18) # see issue 133

  # Wait until debug monitor launch
  sleep(2);

  # Loop over set lists
  for i in range(1, 4):
    # Start round
    msg = Int32()
    msg.data = i
    pub1.publish(msg)

    msg = String()
    msg.data = "Task round {} started".format(i)
    pub2.publish(msg)

    # Loop over subtasks
    for j in range(10):
      msg = String()
      msg.data = "Subtask {} started".format(j)
      pub3.publish(msg)

      # Loop over operations
      for k in range(10):
        msg = String()
        msg.data = "Operation {} started".format(k)
        pub4.publish(msg)

        # Do some operation
        sleep(1)

        msg = String()
        msg.data = "Operation {} finished".format(k)
        pub4.publish(msg)

      msg = String()
      msg.data = "Subtask {} finished".format(j)
      pub3.publish(msg)
