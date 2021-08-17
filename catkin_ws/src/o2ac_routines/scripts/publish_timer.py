#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA
import rospy

def time_convert(sec):
    mins = sec // 60
    sec = int(sec % 60)
    hours = mins // 60
    mins = mins % 60
    return "{:02d}:{:02d}:{:02d}".format(int(hours),int(mins),sec)

def main():
    rospy.init_node("overlay_sample")

    total_time = rospy.get_param("/competition_time", 30.0*60.0)
    start_time = rospy.get_time()

    text_pub = rospy.Publisher("text_sample", OverlayText, queue_size=1)
    rate = 2
    r = rospy.Rate(rate)

    while not rospy.is_shutdown():
        elapse_time = (rospy.get_time()-start_time)
        text = OverlayText()
        text.width = 700
        text.height = 80
        text.left = 10
        text.top = 850
        text.text_size = 40
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = "Time left: %s" % (time_convert(total_time-elapse_time))
        text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        text_pub.publish(text)
        r.sleep()

if __name__ == "__main__":
    main()
