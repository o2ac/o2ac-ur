#!/usr/bin/env python

from o2ac_routines.base import O2ACBase

import sys, signal
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def main():
    controller = O2ACBase()

    controller.execute_manual_routine("bearing_orient_totb")

if __name__ == "__main__":
    main()
