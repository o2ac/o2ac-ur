#!/bin/bash

# This script installs GitLab Runner, its dependencies on the host, and registers one runner instance to GitLab.com.
# Multiple executions of this script add multiple instances. However, a runner is designed to start multiple executors.
# The generated configuration resides in '/etc/gitlab-runner/config.toml', where the amount of concurrent executors can also be set manually.
#
# Usage: bash SETUP-GITLAB-RUNNER.bash <gitlab-runner-registration-token>
#
# <gitlab-runner-registration-token>: To be retrieved from https://gitlab.com/o2ac/o2ac-ur/-/settings/ci_cd.

################################################################################

# Check if the user has provided the mandatory <gitlab-runner-registration-token> argument.
if [[ "$#" -ne 1 ]]; then
  echo "Usage: bash SETUP-GITLAB-RUNNER.sh <gitlab-runner-registration-token>"
  exit 0
fi

################################################################################

# Pin the versions of the core tools and packages for improved stability.
DOCKER_VERSION="5:20.10.5~3-0~ubuntu-$(lsb_release -cs)"
GITLAB_RUNNER_VERSION="13.11.0"

################################################################################

# Install the OpenSSH server missing in the desktop flavors of Ubuntu.
sudo apt-get update
sudo apt-get install -y \
  openssh-server

################################################################################

# Install Docker Community Edition.
# https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

# Remove the older versions of Docker if any.
sudo apt-get remove \
  docker \
  docker-engine \
  docker.io \
  containerd \
  runc

# Gather the required packages for the Docker installation.
sudo apt-get update
sudo apt-get install -y \
  apt-transport-https \
  ca-certificates \
  curl \
  gnupg-agent \
  software-properties-common

# Add the official Docker GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88

# Add the Docker repository.
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# Install Docker version 'DOCKER_VERSION'.
# Any existing installation will be replaced.
sudo apt-get update
sudo apt-get install -y \
  docker-ce=${DOCKER_VERSION} --allow-downgrades \
  docker-ce-cli=${DOCKER_VERSION} \
  containerd.io

# Test the Docker installation after making sure that the service is running.
sudo service docker stop
sudo service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
sudo docker version
sudo docker run --rm hello-world

################################################################################

# Add the current user to the 'docker' group to run Docker without 'sudo'.
# Logging out and back in is required for the group change to take effect.
sudo usermod -a -G docker ${USER}

################################################################################

# Install GitLab Runner.
# https://docs.gitlab.com/runner/
# https://docs.gitlab.com/runner/install/linux-repository.html

# Add the GitLab official repository.
curl -L https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh | sudo bash

# Install GitLab Runner version 'GITLAB_RUNNER_VERSION'.
sudo apt-get update
sudo apt-get install -y \
  gitlab-runner=${GITLAB_RUNNER_VERSION}

# Register a new runner to GitLab.com using the CLI interface.
# The configuration is generated into '/etc/gitlab-runner/config.toml'.
# For multiple executors on this runner, edit the executor variable in the configuration file.
# https://docs.gitlab.com/runner/register/index.html
# https://docs.gitlab.com/runner/register/index.html#one-line-registration-command
sudo gitlab-runner register \
  --non-interactive \
  --url "https://gitlab.com/" \
  --registration-token "$1" \
  --executor "docker" \
  --docker-privileged=true \
  --docker-image "alpine:latest" \
  --docker-volumes "/opt/gitlab-runner/:/data/:rw" \
  --env "DOCKER_DRIVER=overlay2" \
  --env "DOCKER_TLS_CERTDIR=" \
  --description "${HOSTNAME}-${RANDOM}"

# Reload the 'gitlab-runner' configuration.
sudo killall -SIGHUP gitlab-runner

# Make sure that the OverlayFS driver is loaded on boot.
# https://docs.gitlab.com/ce/ci/docker/using_docker_build.html#use-driver-for-every-project
sudo sh -c \
  'grep -q "overlay" /etc/modules \
  || echo "\n# Make sure OverlayFS driver is loaded on boot.\noverlay" >> /etc/modules'

