#!/bin/bash

################################################################################

# Manually install the Parallels Tools inside the Docker container.
# https://kb.parallels.com/en/121370
mkdir -p /media/cdrom/
mount /dev/sr0 /media/cdrom/
cd /media/cdrom/
./install --install-unattended-with-deps --verbose
cd /root/
