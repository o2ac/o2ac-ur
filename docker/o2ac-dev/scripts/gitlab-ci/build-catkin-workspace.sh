#!/bin/bash

# Terminate the script after the first failure and return non-zero exit code.
set -e

################################################################################

# Clone the project repository in 'root' folder.
cd /root/o2ac-ur/ && git clone $CI_REPOSITORY_URL

# Checkout the commit pushed to GitLab CI.
cd /root/$CI_PROJECT_NAME/ && git checkout $CI_COMMIT_SHA

# TODO: Pass SSH key with a GitLab CI secret variable.
# https://docs.gitlab.com/ee/ci/ssh_keys/
# Install the SSH deploy key of the GitLab repository.
mkdir -p /root/.ssh/
chmod 700 /root/.ssh/
cp /root/$CI_PROJECT_NAME/.ssh/o2ac-ur-id_rsa /root/.ssh/
chmod 600 /root/.ssh/o2ac-ur-id_rsa
cat <<'EOF' >> /root/.ssh/config;
Host gitlab.com
  Hostname gitlab.com
  IdentityFile /root/.ssh/o2ac-ur-id_rsa
  StrictHostKeyChecking no
  User git
EOF

# Download and initialize all the Git submodules recursively.
# https://git-scm.com/book/en/v2/Git-Tools-Submodules
cd /root/$CI_PROJECT_NAME/ && git submodule update --init --recursive

# Source the ROS environment.
source /opt/ros/kinetic/setup.bash

# Initialize the Catkin workspace.
cd /root/$CI_PROJECT_NAME/catkin_ws/ && catkin_init_workspace

# Build the Catkin workspace.
cd /root/$CI_PROJECT_NAME/catkin_ws/ && catkin build
