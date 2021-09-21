#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
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
#  * Neither the name of OMRON SINIC X nor the names of its
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
# Author: Felix von Drigalski, Cristian C. Beltran-Hernandez

# This file publishes a countdown timer (visualized in Rviz).

try:
    from jsk_rviz_plugins.msg import *
except:
    import roslib
    roslib.load_manifest("jsk_rviz_plugins")
    from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA
import rospy


def time_convert(sec):
    mins = sec // 60
    sec = int(sec % 60)
    hours = mins // 60
    mins = mins % 60
    return "{:02d}:{:02d}:{:02d}".format(int(hours), int(mins), sec)


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
