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
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/o2ac-ur/catkin_ws/src/
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/o2ac-ur/underlay_ws/src/

################################################################################

# Define Bash functions to conveniently execute the helper scripts in the current shell process.

function o2ac-repair-git-paths () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  popd
}

function o2ac-build-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/fix-permission-issues.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/initialize-catkin-workspace.sh
  popd
}

function o2ac-initialize-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/fix-permission-issues.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/initialize-catkin-workspace.sh
  popd
}

function o2ac-reset-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/fix-permission-issues.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/reset-catkin-workspace.sh
  popd
}

function o2ac-fix-permission-issues () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/fix-permission-issues.sh
  popd
}

function o2ac-tests () {
  # Run tests.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/run-tests.sh
  popd
}

function o2ac-run-vscode-editor () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/run-vscode-editor.sh
  popd
}

function o2ac-get-fully-started () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/repair-git-paths.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/fix-permission-issues.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/reset-catkin-workspace.sh
  source /root/o2ac-ur/docker/o2ac-dev/scripts/run-vscode-editor.sh
  popd
}

function o2ac-magic-rosdep-command () {
  pushd .
  source /root/o2ac-ur/docker/o2ac-dev/scripts/magic-rosdep.sh
  popd
}

function dispurdf () {
  roslaunch urdf_tutorial display.launch model:=$1
}

function axclient () {
  rosrun actionlib axclient.py $1
}

###############################################################################

# aliases

alias cw='cd /root/o2ac-ur/catkin_ws'
alias cs='cd /root/o2ac-ur/catkin_ws/src'
alias cb='catkin build'
alias cc='catkin clean'
alias s='source /root/o2ac-ur/catkin_ws/devel/setup.bash'

alias ucw='cd /root/o2ac-ur/underlay_ws'
alias ucs='cd /root/o2ac-ur/underlay_ws/src'
alias us='source /root/o2ac-ur/underlay_ws/devel/setup.bash'

alias rd='rosdep install -i --from-paths'
alias rqt_plot='rosrun rqt_plot rqt_plot'
alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'
alias rqt_tf_tree='rosrun rqt_tf_tree rqt_tf_tree'

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
cd /root/o2ac-ur/
