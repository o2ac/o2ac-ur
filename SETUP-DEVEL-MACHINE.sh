#!/bin/bash

################################################################################

# Setup global environment that requires 'sudo' privileges.
sudo -E ./INCL-SUDO-ENV.sh

################################################################################

# Setup local environment that does not require 'sudo' privileges.
./INCL-USER-ENV.sh

echo 'Please reboot your machine.'