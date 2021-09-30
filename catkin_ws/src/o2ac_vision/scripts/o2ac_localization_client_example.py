#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Science and Technology
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
#  * Neither the name of National Institute of Advanced Science and Technology nor the names of its
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
import actionlib
from o2ac_msgs import msg as omsg
from aist_model_spawner import ModelSpawnerClient

######################################################################
#  global functions                                                  #
######################################################################


def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True


#########################################################################
#  class ObjectLocalizationClient                                        #
#########################################################################


class ObjectLocalizationClient(object):
    def __init__(self, server=""):
        super(ObjectLocalizationClient, self).__init__()

        self._localize = actionlib.SimpleActionClient(
            server + "/localize_object", omsg.localizeObjectAction
        )
        self._localize.wait_for_server()

    def send_goal(self, model):
        self._localize.send_goal(omsg.localizeObjectGoal(item_id=model))

    def wait_for_result(self, timeout=rospy.Duration()):
        if not self._localize.wait_for_result(timeout):
            self._localize.cancel_goal()  # Cancel goal if timeout expired

    def get_result(self):
        result = self._localize.get_result()
        return result.succeeded, result.confidences, result.detected_poses


#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == "__main__":

    rospy.init_node("o2ac_localization_client")

    _Models = (
        "01-BASE",  # 1
        "02-PANEL",  # 2
        "03-PANEL2",  # 3
        "04_37D-GEARMOTOR-50-70",  # 4
        "05_MBRFA30-2-P6",  # 5
        "06_MBT4-400",  # 6
        "07_SBARB6200ZZ_30",  # 7
        "08_KZAF1075NA4WA55GA20AA0",  # 8
        "09_EDCS10",  # 9
        "10_CLBPS10_17_4",  # 10
        "11_MBRAC60-2-10",  # 11
        "12_CLBUS6-9-9.5",  # 12
        "13_MBGA30-2",  # 13
        "14_BGPSL6-9-L30-F8",  # 14
    )

    recognizer = ObjectLocalizationClient("o2ac_vision_server")
    spawner = ModelSpawnerClient()

    while not rospy.is_shutdown():
        print("\n=============")
        for i in range(len(_Models)):
            print("{:4}: {}".format(i + 1, _Models[i]))
        print("   d: delete all models spawned")
        print("   q: quit")

        key = raw_input("\nEnter object ID >> ")

        if key == "q":
            break
        elif key == "d":
            spawner.delete_all()
        elif is_num(key):
            id = int(key)

            if id - 1 in range(len(_Models)):
                model = _Models[id - 1]
                recognizer.send_goal(model)
                recognizer.wait_for_result()
                success, confidences, poses = recognizer.get_result()
                if success:
                    print("  Succeeded with confidence={}".format(confidences))
                    for i, pose in enumerate(poses):
                        spawner.add(model, pose, "_{:02d}_".format(i))
                else:
                    print("  Failed")
            else:
                print("  Unknown object ID[{}]".format(id))
        else:
            print("  Unknown command[{}]".format(key))
