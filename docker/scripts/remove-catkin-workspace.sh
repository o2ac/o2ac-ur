#!/bin/bash

################################################################################

# Remove the Catkin workspace.
cd /root/catkin_ws/ && catkin clean -y
cd /root/catkin_ws/ && rm -r CMakeLists.txt .catkin_tools/

# TODO: Find a way to "unsource" without opening a new shell.
# Unsource the Catkin workspace.
exec bash
