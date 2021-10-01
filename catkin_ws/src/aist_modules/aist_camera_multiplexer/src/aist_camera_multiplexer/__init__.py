# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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
# Author: Toshio Ueshiba
#
import rospy
import dynamic_reconfigure.client

######################################################################
#  class CameraMultiplexerClient                                     #
######################################################################
class CameraMultiplexerClient(object):
    def __init__(self, server='camera_multiplexer'):
        super(CameraMultiplexerClient, self).__init__()
        self._camera_names = rospy.get_param(server + '/camera_names', [])
        self._dyn_reconf   = dynamic_reconfigure.client.Client(server,
                                                               timeout=5.0)

    @property
    def camera_names(self):
        return self._camera_names

    def active_camera(self):
        conf = self._dyn_reconf.get_configuration()
        return self._camera_names[conf['active_camera']]

    def activate_camera(self, camera_name):
        self._dyn_reconf.update_configuration(
            {'active_camera': self._camera_names.index(camera_name)})
        rospy.sleep(0.2)

######################################################################
#  class RealSenseMultiplexerClient                                  #
######################################################################
class RealSenseMultiplexerClient(CameraMultiplexerClient):

    class RealSenseCamera(object):
        def __init__(self, server):
            super(RealSenseMultiplexerClient.RealSenseCamera, self).__init__()
            self._dyn_camera = dynamic_reconfigure.client.Client(server,
                                                                 timeout=5.0)
            self._dyn_sensor = dynamic_reconfigure.client.Client(
                                server + '/coded_light_depth_sensor',
                                timeout=5.0)
            self.laser_power = 16
            self._recent_laser_power = self.laser_power

        @property
        def laser_power(self):
            conf = self._dyn_sensor.get_configuration()
            return conf['laser_power']

        @laser_power.setter
        def laser_power(self, value):
            self._dyn_sensor.update_configuration({'laser_power': value})

        def enable_laser(self, enabled):
            if enabled:
                self.laser_power = self._recent_laser_power
            else:
                self._recent_laser_power = self.laser_power
                self.laser_power = 0
            rospy.sleep(0.2)

        def continuous_shot(self, enabled):
            self._dyn_camera.update_configuration({'enable_streaming': enabled})
            rospy.sleep(0.2)
            return True

    def __init__(self, server='camera_multiplexer'):
        try:
            super(RealSenseMultiplexerClient, self).__init__(server)
            self._cameras = dict(zip(self.camera_names,
                                 [RealSenseMultiplexerClient.RealSenseCamera(
                                     camera_name)
                                  for camera_name in self.camera_names]))
        except:
            rospy.logerr("Cameras failed to initialize. "
            "Are the camera nodes started? Does /camera_multiplexer/camera_names"
            "contain unused cameras? camera_names: " + str(self._camera_names))
        for camera in self._cameras.values():
            camera.enable_laser(False)
        self._cameras[self.active_camera()].enable_laser(True)

    def activate_camera(self, camera_name):
        self._cameras[self.active_camera()].enable_laser(False)
        super(RealSenseMultiplexerClient, self).activate_camera(camera_name)
        self._cameras[self.active_camera()].enable_laser(True)
