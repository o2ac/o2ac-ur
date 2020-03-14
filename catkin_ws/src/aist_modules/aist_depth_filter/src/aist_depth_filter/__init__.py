import rospy
import dynamic_reconfigure.client
from std_srvs import srv as ssrv

#########################################################################
#  class DepthFilterClient                                              #
#########################################################################
class DepthFilterClient(object):
    def __init__(self, name="depth_filter"):
        super(DepthFilterClient, self).__init__()
        self._saveBG     = rospy.ServiceProxy(name + "/saveBG",  ssrv.Trigger)
        self._savePly    = rospy.ServiceProxy(name + "/savePly", ssrv.Trigger)
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)

    def saveBG(self):
        return self._saveBG().success

    def savePly(self):
        return self._savePly().success

    def set_background_threshold(self, thresh_bg):
        self._dyn_reconf.update_configuration({"thresh_bg": thresh_bg})
        return True

    def set_depth_range(self, near, far):
        self._dyn_reconf.update_configuration({"near": near, "far": far})
        return True

    def set_roi(self, top, bottom, left, right):
        self._dyn_reconf.update_configuration({"top" : top,  "bottom": bottom,
                                               "left": left, "right" : right})
        return True

    def set_scale(self, scale):
        self._dyn_reconf.update_configuration({"scale": scale})
        return True

    def set_window_radius(self, window_radius):
        self._dyn_reconf.update_configuration({"window_radius": window_radius})
        return True
