#!/bin/bash

################################################################################

# Install the SSH deploy key of the GitLab repository.
mkdir -p ~/.ssh/
chmod 700 ~/.ssh/
cp ./.ssh/o2ac-ur-id_rsa ~/.ssh/
chmod 600 ~/.ssh/o2ac-ur-id_rsa
if grep -E -q '^Host gitlab\.com' ~/.ssh/config; then
  echo "SSH configuration for 'gitlab.com' is already included in '~/.ssh/config'."
  echo "Manually add 'IdentityFile ~/.ssh/o2ac-ur-id_rsa' under 'gitlab.com' if the connection is refused."
else
  echo "Add new SSH configuration for 'gitlab.com' in '~/.ssh/config'.";
  cat <<'EOF' >> ~/.ssh/config;
Host gitlab.com
  Hostname gitlab.com
  IdentityFile ~/.ssh/o2ac-ur-id_rsa
  StrictHostKeyChecking no
  User git
EOF
fi

################################################################################

# Modify the HTTPS URL of the Git remote origin to use SSH:
# Enable authentication by using '~/.ssh/o2ac-ur-id_rsa' instead of a GitLab account.
GIT_HTTPS_URL=`git config --get remote.origin.url`
GIT_SSH_URL=`echo ${GIT_HTTPS_URL} | sed -e 's/^https\{0,1\}:/ssh:/'`
if [ "${GIT_HTTPS_URL}" != "${GIT_SSH_URL}" ]; then
  echo "Update the Git remote origin from '${GIT_HTTPS_URL}' to '${GIT_SSH_URL}'."
  echo "Execute 'git config --get remote.origin.url' to manually confirm if needed."
  git remote set-url origin ${GIT_SSH_URL}
fi

################################################################################

# Download and initialize all the Git submodules recursively.
# https://git-scm.com/book/en/v2/Git-Tools-Submodules
git submodule update --init --recursive

################################################################################

# Setup the Bash shell environment with '.bashrc'.

# Force color prompt in terminal.
sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc

# Set default Docker runtime to use in './docker/docker-compose.yml'.
if ! grep -q "export DOCKER_RUNTIME" ~/.bashrc; then
  if [ -e /proc/driver/nvidia/version ]; then
    DOCKER_RUNTIME=nvidia
  else
    DOCKER_RUNTIME=runc
  fi
  cat <<EOF >> ~/.bashrc

# Set default Docker runtime to use in '~/o2ac-ur/docker/docker-compose.yml':
# 'runc' (Docker default) or 'nvidia' (Nvidia Docker 2).
export DOCKER_RUNTIME=${DOCKER_RUNTIME}
EOF
fi
