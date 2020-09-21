#!/usr/bin/env python

import rospy, actionlib
from o2ac_msgs          import msg as omsg
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
#  class ObjectRecognitionClient                                        #
#########################################################################
class ObjectRecognitionClient(object):

    def __init__(self, server=''):
        super(ObjectRecognitionClient, self).__init__()

        self._recognize = actionlib.SimpleActionClient(server +
                                                       '/recognize_object',
                                                       omsg.detectObjectAction)
        self._recognize.wait_for_server()

    def send_goal(self, model):
        self._recognize.send_goal(omsg.detectObjectGoal(item_id=model))

    def wait_for_result(self, timeout=rospy.Duration()):
        if (not self._recognize.wait_for_result(timeout)):
            self._recognize.cancel_goal()  # Cancel goal if timeout expired

    def get_result(self):
        result = self._recognize.get_result()
        return result.succeeded, result.confidence, result.detected_pose

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('o2ac_recognition_client')

    _Models = ('01-BASE',                       # 1
               '02-PANEL',                      # 2
               '03-PANEL2',                     # 3
               '04_37D-GEARMOTOR-50-70',        # 4
               '05_MBRFA30-2-P6',               # 5
               '06_MBT4-400',                   # 6
               '07_SBARB6200ZZ_30',             # 7
               '08_KZAF1075NA4WA55GA20AA0',     # 8
               '09_EDCS10',                     # 9
               '10_CLBPS10_17_4',               # 10
               '11_MBRAC60-2-10',               # 11
               '12_CLBUS6-9-9.5',               # 12
               '13_MBGA30-2',                   # 13
               '14_BGPSL6-9-L30-F8',            # 14
              )

    recognizer = ObjectRecognitionClient('object_recognizer')
    spawner    = ModelSpawnerClient()

    while not rospy.is_shutdown():
        print('\n=============')
        for i in range(len(_Models)):
            print("{:4}: {}".format(i+1, _Models[i]))
        print('   d: delete all models spawned')
        print('   q: quit')

        key = raw_input('\nEnter object ID >> ')

        if key == 'q':
            break
        elif key == 'd':
            spawner.delete_all()
        elif (is_num(key)):
            id = int(key)

            if id-1 in range(len(_Models)):
                model = _Models[id-1]
                recognizer.send_goal(model)
                recognizer.wait_for_result()
                success, confidence, pose = recognizer.get_result()
                if success:
                    print("  Succeeded with confidence={}".format(confidence))
                    spawner.add(model, pose)
                else:
                    print("  Failed")
            else:
                print("  Unknown object ID[{}]".format(id))
        else:
            print("  Unknown command[{}]".format(key))
