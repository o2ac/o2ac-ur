#!/bin/bash

DOCKER_VERSION="5:18.09.3~3-0~ubuntu-$(lsb_release -cs)"
GITLAB_RUNNER_VERSION="11.8.0"

################################################################################

# Install OpenSSH server missing in the desktop flavors of Ubuntu.
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
  docker-ce=${DOCKER_VERSION} \
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

# Register the GitLab Runner using a configuration file.
# The initial registration still needs to be manual to obtain the 'token' of the TOML file.
# https://docs.gitlab.com/runner/register/index.html
# https://docs.gitlab.com/runner/register/index.html#one-line-registration-command
case "$1" in
  ( "" )
  sudo cp ./gitlab-ci/config/o2ac-runner-01.toml /etc/gitlab-runner/config.toml
  ;;
  ( "o2ac-runner-01" | \
    "o2ac-runner-02" )
  sudo cp ./gitlab-ci/config/$1.toml /etc/gitlab-runner/config.toml
  ;;
  ( * )
  echo "Failed to register the GitLab Runner: '$1' is not a valid argument value."
  ;;
esac

# Make sure OverlayFS driver is loaded on boot.
# https://docs.gitlab.com/ce/ci/docker/using_docker_build.html#use-driver-for-every-project
sudo sh -c \
  'grep -q "overlay" /etc/modules \
  || echo "\n# Make sure OverlayFS driver is loaded on boot.\noverlay" >> /etc/modules'
