#!/usr/bin/env python

import rospy, copy
from std_srvs.srv                 import Empty, Trigger
from aist_handeye_calibration.srv import GetSampleList, ComputeCalibration
from aist_handeye_calibration     import HandEyeCalibrationBaseRoutines

######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(HandEyeCalibrationBaseRoutines):
    def __init__(self):
        super(HandEyeCalibrationRoutines, self).__init__()

        self._eye_on_hand = rospy.get_param('~eye_on_hand', False)
        self._initpose    = rospy.get_param('~initpose', [])
        self._keyposes    = rospy.get_param('~keyposes', [])
        self._sleep_time  = rospy.get_param('~sleep_time', 1)

        if rospy.get_param('calibration', True):
            ns = '/handeye_calibrator'
            self.get_sample_list \
                = rospy.ServiceProxy(ns + '/get_sample_list', GetSampleList)
            self.take_sample \
                = rospy.ServiceProxy(ns + '/take_sample', Trigger)
            self.compute_calibration \
                = rospy.ServiceProxy(ns + '/compute_calibration',
                                     ComputeCalibration)
            self.save_calibration \
                = rospy.ServiceProxy(ns + '/save_calibration', Trigger)
            self.reset = rospy.ServiceProxy(ns + '/reset', Empty)
        else:
            self.get_sample_list     = None
            self.take_sample         = None
            self.compute_calibration = None
            self.save_calibration    = None
            self.reset               = None

    def move_to(self, pose, keypose_num, subpose_num):
        success = self.move(pose, True)
        if not success:
            return False

        if self.take_sample:
            try:
                rospy.sleep(self._sleep_time)  # Wait for the robot to settle.
                self.trigger_frame()
                res = self.take_sample()

                n = len(self.get_sample_list().cMo)
                print('  {} samples taken: {}').format(n, res.message)
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s' % e)
                success = False

        return success

    def move_to_subposes(self, pose, keypose_num):
        subpose = copy.copy(pose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 1))
            if self.move_to(subpose, keypose_num, i + 1):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 1))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 1))
            subpose[3] -= 30

        subpose[3] = roll - 30
        subpose[4] += 15

        for i in range(2):
            print('\n--- Subpose [{}/5]: Try! ---'.format(i + 4))
            if self.move_to(subpose, keypose_num, i + 4):
                print('--- Subpose [{}/5]: Succeeded. ---'.format(i + 4))
            else:
                print('--- Subpose [{}/5]: Failed. ---'.format(i + 4))
            subpose[4] -= 30

    def run(self):
        if self.reset:
            self.reset()

        # Reset pose
        self.go_to_named_pose('home')
        self.move(self._initpose, True)

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [{}/{}]: Try! ***'
                  .format(i, len(keyposes)))
            if self._eye_on_hand:
                self.move_to(keypose, i, 1)
            else:
                self.move_to_subposes(keypose, i)
            print('*** Keypose [{}/{}]: Completed. ***'
                  .format(i, len(keyposes)))

        if self.compute_calibration:
            res = self.compute_calibration()
            print(res.message)
            res = self.save_calibration()
            print(res.message)

        self.go_to_named_pose('home')


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    calibrate = HandEyeCalibrationRoutines()
    while not rospy.is_shutdown():
        if raw_input('Hit return key to start >> ') == 'q':
            break
        calibrate.run()
