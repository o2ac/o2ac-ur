#! /usr/bin/env python

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
#  * Neither the name of Team o2ac nor the names of its
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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski


from time import sleep, time
import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge         import CvBridge

import numpy as np
import cv2


class TextImageWriter():
  def __init__(self):
    self.img = np.zeros((100,1000,3), np.uint8)
    self.sub_text = rospy.Subscriber("/o2ac_text_to_image", String, self.write_text_and_pub)
    self.pub_img = rospy.Publisher("/o2ac_status_text_image", Image, queue_size=1)
    self.bridge = CvBridge()
    rospy.sleep(1.0)
  
  # https://stackoverflow.com/questions/52846474/how-to-resize-text-for-cv2-puttext-according-to-the-image-size-in-opencv-python
  def get_optimal_font_scale(self, text, image_width, image_height):
    for scale in reversed(range(0, 60, 1)):
      textSize = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=scale/10.0, thickness=1)
      # print("textSize")
      # print(textSize)
      text_width = textSize[0][0]
      text_height = textSize[0][1]
      # print(text_width, text_height)
      if (text_width <= image_width - 20) and (text_height <= image_height - 40):
          return scale/10.0
    return 1

  # https://stackoverflow.com/questions/16615662/how-to-write-text-on-a-image-in-windows-using-python-opencv2
  def write_text_and_pub(self, text_message):
    # Write some Text
    text = text_message.data
    rospy.loginfo("Received text to convert: " + text)
    
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,self.img.shape[0]-30)
    lineThickness          = 5
    lineType               = cv2.LINE_AA
    fontScale              = cv2.getFontScaleFromHeight(font, self.img.shape[0]-40, lineThickness)
    # fontScale              = self.get_optimal_font_scale(text, self.img.shape[1], self.img.shape[0])
    # fontScale              = 1
    fontColor              = (0,255,255)

    self.img = np.zeros(self.img.shape, np.uint8)
    cv2.putText(self.img, text, 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        lineThickness,
        lineType)

    # # Display the image
    # cv2.imshow("img",self.img)
    # cv2.waitKey(500)
    rospy.sleep(.4)  # Apparently required for the image to be filled

    # Publish the image
    self.imgmsg = self.bridge.cv2_to_imgmsg(self.img, encoding='passthrough')
    self.pub_img.publish(self.imgmsg)
    rospy.loginfo("Published image.")

if __name__ == "__main__":
  # Initialize the ROS node
  rospy.init_node("o2ac_text_to_image_converter")
  c = TextImageWriter()
  rospy.loginfo("O2AC text to image writer started up")
  rospy.spin()
