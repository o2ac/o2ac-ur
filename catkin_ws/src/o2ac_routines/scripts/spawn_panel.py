#!/usr/bin/env python

import signal
from ur_control.constants import TERMINATION_CRITERIA
import o2ac_routines.helpers as helpers
from o2ac_routines.assembly import O2ACAssembly
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_assembly_database.parts_reader import PartsReader
import moveit_msgs.msg
from o2ac_msgs.srv import *
import numpy as np
import time
import math
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_msgs
import tf_conversions
import tf
from math import pi, radians, sin, cos
tau = 2.0*pi  # Part of math from Python 3.6


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node('o2ac_routines', anonymous=False)
    c = O2ACAssembly()
    c.spawn_panel()
