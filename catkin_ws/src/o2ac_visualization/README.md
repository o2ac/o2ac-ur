# o2ac_visualization

This package is meant to visualize the state of the system, both for easier debugging and for demoing.

It needs to be reworked, as it is on the state of the WRS2018. It used an OpenCV image to display the system information, but we will shift it to Rviz instead.

## Reset time to the current time
rosservice call /o2ac_visualization/reset_timer
