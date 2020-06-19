#!/usr/bin/env python
import socket
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped

def WrenchPublish():
    topic_name = rospy.get_param("ft300data/topic_name", "ft300wrench")
    pub = rospy.Publisher(topic_name, WrenchStamped, queue_size=100)
    rospy.init_node('FT300WrenchData', anonymous=True)
    rate = rospy.Rate(100)
    wrench = WrenchStamped()
    HOST = rospy.get_param("ft300data/robot_ip", "192.168.1.42") # The remote host (UR controller)
    PORT = 63351    # The port that the FT300 sensor streams its data to
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    while not rospy.is_shutdown():
        data = s.recv(1024)
        data = data.replace("(","")
        data = data.replace(")","")
        datasplit=data.split(",")
        if len(datasplit) == 6:    # To avoid termination (since sometimes during impact, the UR apparently skips a comma)
            datasplit= [float(i) for i in datasplit]
            wrench.wrench.force.x=datasplit[0]
            wrench.wrench.force.y=datasplit[1]
            wrench.wrench.force.z=datasplit[2]
            wrench.wrench.torque.x=datasplit[3]
            wrench.wrench.torque.y=datasplit[4]
            wrench.wrench.torque.z=datasplit[5]
            wrench.header=Header()
            wrench.header.stamp=rospy.Time.now()
            # rospy.logdebug(wrench)
            pub.publish(wrench)
        rate.sleep()
    s.close

if __name__ == '__main__':
    try:
        WrenchPublish()
    except rospy.ROSInterruptException:
        pass
