#!/usr/bin/env python

import rospy
from geometry_msgs            import msg as gmsg
from aist_handeye_calibration import HandEyeCalibrationBaseRoutines

######################################################################
#  class CheckCalibrationRoutines                                    #
######################################################################
class CheckCalibrationRoutines(HandEyeCalibrationBaseRoutines):
    def __init__(self):
        super(CheckCalibrationRoutines, self).__init__()
        self._effector_frame = ''       # Move tooltip to the target pose

    def move_to_marker(self):
        self.trigger_frame()
        marker_pose = rospy.wait_for_message(self._camera_name
                                             + '/aruco_detector/pose',
                                             gmsg.PoseStamped, 10)
        approach_pose = self.effector_target_pose(marker_pose, (0, 0, 0.05))

        #  We have to transform the target pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        target_pose = self.transform_pose_to_reference_frame(
                          self.effector_target_pose(marker_pose, (0, 0, 0)))
        self.move(approach_pose, True)
        rospy.sleep(1)
        return self.move(target_pose, True)

    def run(self):
        self.go_to_named_pose('home')

        while not rospy.is_shutdown():
            try:
                key = raw_input('>> ')
                if key == 'q':
                    break
                elif key == 'h':
                    self.go_to_named_pose('home')
                else:
                    self.move_to_marker()
            except rospy.ROSException as ex:
                print ex.message
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

        self.go_to_named_pose('home')

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    routines = CheckCalibrationRoutines()
    routines.run()
