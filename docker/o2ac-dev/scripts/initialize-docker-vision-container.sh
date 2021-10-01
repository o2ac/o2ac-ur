#!/bin/bash

################################################################################

# Start the D-Bus daemon.
service dbus start

################################################################################

export ROS_HOSTNAME=o2ac-vision-pc
export ROS_MASTER_URI=http://o2ac-vision-pc:11311

################################################################################

# Support Intel 3D acceleration (when no Nvidia GPU drivers are found).

# Remove the configuraion files related to the GL library provided by Nvidia and re-generate '/etc/ld.so.cache' by executing 'ldconfig'.
# This is required to avoid problems occurring when the host system does not have the GL library provided by Nvidia.
if [ "$DOCKER_RUNTIME" = "runc" ]; then
  rm /etc/ld.so.conf.d/nvidia.conf /etc/ld.so.conf.d/glvnd.conf;
  ldconfig;
fi

################################################################################

# Keep the Docker container running in the background.
# https://stackoverflow.com/questions/30209776/docker-container-will-automatically-stop-after-docker-run-d
tail -f /dev/null
