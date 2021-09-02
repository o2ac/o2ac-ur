#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_msgs
import tf_conversions
import tf
from math import pi, radians, sin, cos
tau = 2.0*pi  # Part of math from Python 3.6
import math
import time
import numpy as np

from o2ac_msgs.srv import *
import moveit_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.assembly import O2ACAssembly
import o2ac_routines.helpers as helpers

from ur_control.constants import TERMINATION_CRITERIA

import sys, signal
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node('o2ac_routines', anonymous=False)
    c = O2ACAssembly()
    c.spawn_panel()
