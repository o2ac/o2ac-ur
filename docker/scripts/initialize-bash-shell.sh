#!/bin/bash

################################################################################

# Link the default shell 'sh' to Bash.
alias sh='/bin/bash'

################################################################################

# Configure the terminal.

# Disable flow control. If enabled, inputting 'ctrl+s' locks the terminal until inputting 'ctrl+q'.
stty -ixon

################################################################################

# Configure 'umask' for giving read/write/execute permission to group members.
umask 0002

################################################################################

# Add the Catkin workspaces to the 'ROS_PACKAGE_PATH'.
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/catkin_ws/src/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/underlay_ws/src/

################################################################################

# Define Bash functions to conveniently execute the helper scripts in the current shell process.

function o2ac-repair-git-paths () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  popd
}

function o2ac-initialize-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  source /root/docker/scripts/fix-permission-issues.sh
  source /root/docker/scripts/initialize-catkin-workspace.sh
  popd
}

function o2ac-reset-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  source /root/docker/scripts/fix-permission-issues.sh
  source /root/docker/scripts/reset-catkin-workspace.sh
  popd
}

function o2ac-fix-permission-issues () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  source /root/docker/scripts/fix-permission-issues.sh
  popd
}

function o2ac-run-vscode-editor () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  source /root/docker/scripts/run-vscode-editor.sh
  popd
}

function o2ac-get-fully-started () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/docker/scripts/repair-git-paths.sh
  source /root/docker/scripts/fix-permission-issues.sh
  source /root/docker/scripts/reset-catkin-workspace.sh
  source /root/docker/scripts/run-vscode-editor.sh
  popd
}

function o2ac-magic-rosdep-command () {
  pushd .
  source /root/docker/scripts/magic-rosdep.sh
  popd
}

###############################################################################

# aliases

alias cw='cd /root/catkin_ws'
alias cs='cd /root/catkin_ws/src'
alias cb='catkin build'
alias setup='source /root/catkin_ws/devel/setup.bash'

alias ucw='cd /root/underlay_ws'
alias ucs='cd /root/underlay_ws/src'
alias usetup='source /root/underlay_ws/devel/setup.bash'

###############################################################################

# PhoXi settings

if [ -d /opt/PhotoneoPhoXiControl-1.2.14 ]; then
  export PHOXI_CONTROL_PATH=/opt/PhotoneoPhoXiControl-1.2.14
  export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
  export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
fi

###############################################################################

# PhoLocalizationSDK settings

if [ -d /opt/PhotoneoSDK/Localization ]; then
  export PHO_LOCALIZATION_PATH=/opt/PhotoneoSDK/Localization
  export PATH=${PATH}:${PHO_LOCALIZATION_PATH}/bin
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHO_LOCALIZATION_PATH}/lib
  export CPATH=${CPATH}:${PHO_LOCALIZATION_PATH}/include
fi

################################################################################

# Move to the working directory.
cd /root/
