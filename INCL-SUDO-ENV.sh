#!/bin/bash

DOCKER_VERSION="5:19.03.1~3-0~ubuntu-$(lsb_release -cs)"
DOCKER_COMPOSE_VERSION="1.23.2"
NVIDIA_DOCKER_VERSION="2.2.0-1"
NVIDIA_RUNTIME_VERSION="3.1.0-1"

################################################################################

# Pass 'sudo' privileges if previously granted in parent scripts.
if [ ! -z "$SUDO_USER" ]; then
  export USER=$SUDO_USER
fi

################################################################################

# Install Docker Community Edition.
# https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

# Remove older versions of Docker if any.
apt-get remove \
  docker \
  docker-engine \
  docker.io \
  containerd \
  runc

# Gather required packages for Docker installation.
apt-get update && apt-get install -y \
  apt-transport-https \
  ca-certificates \
  curl \
  gnupg \
  software-properties-common

# Add the official Docker GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
apt-key fingerprint 0EBFCD88

# Add the Docker repository.
add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# Install Docker version 'DOCKER_VERSION'.
# Any existing installation will be replaced.
apt-get update && apt-get install -y \
  docker-ce=${DOCKER_VERSION} \
  docker-ce-cli=${DOCKER_VERSION} \
  containerd.io

# Test the Docker installation after making sure that the service is running.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
docker version
docker run --rm hello-world

################################################################################

# Add the current user to the 'docker' group to run Docker without 'sudo'.
# Logging out and back in is required for the group change to take effect.
usermod -a -G docker ${USER}
echo "Added the current user '${USER}' to the 'docker' group."

# Configure the host system so that 'adduser' command adds future new users to the 'docker' group automatically.
# This enables new users to set up their environment without 'sudo' by only executing 'INCL-USER-ENV.sh'.
ADDUSER_CONFIG=/etc/adduser.conf
if [ ! -f ${ADDUSER_CONFIG} ]; then
  echo "Failed to add future new users to the 'docker' group because the system configuration file '${ADDUSER_CONFIG}' was not found."
else
  if ! grep -q "#EXTRA_GROUPS=\"dialout cdrom floppy audio video plugdev users\"" ${ADDUSER_CONFIG}; then
    echo "Failed to add future new users to the 'docker' group because 'EXTRA_GROUPS' in '${ADDUSER_CONFIG}' has already been customized."
  else
    sed -i 's/#EXTRA_GROUPS="dialout cdrom floppy audio video plugdev users"/EXTRA_GROUPS="dialout cdrom floppy audio video plugdev users docker"/' ${ADDUSER_CONFIG}
    sed -i 's/#ADD_EXTRA_GROUPS=1/ADD_EXTRA_GROUPS=1/' ${ADDUSER_CONFIG}
    echo "Modified '${ADDUSER_CONFIG}' to add all future new users to the 'docker' group upon creation."
  fi
fi

################################################################################

# Install Docker Compose.
# https://docs.docker.com/compose/install/#install-compose
# https://github.com/docker/compose/releases

# Install Docker Compose version 'DOCKER_COMPOSE_VERSION'.
curl -L "https://github.com/docker/compose/releases/download/${DOCKER_COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

# Install Bash command completion for Docker Compose version 'DOCKER_COMPOSE_VERSION'.
curl -L https://raw.githubusercontent.com/docker/compose/${DOCKER_COMPOSE_VERSION}/contrib/completion/bash/docker-compose -o /etc/bash_completion.d/docker-compose

# Test the Docker Compose installation.
docker-compose version

################################################################################

# Install Nvidia Docker 2.
# https://github.com/NVIDIA/nvidia-docker
# https://github.com/NVIDIA/nvidia-docker/wiki/Usage
# https://github.com/nvidia/nvidia-container-runtime#environment-variables-oci-spec

# Remove 'nvidia-docker' and all existing GPU containers.
docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
apt-get purge -y nvidia-docker

# Add the Nvidia Docker package repositories.
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list

# Install 'nvidia-docker2' version 'NVIDIA_DOCKER_VERSION' and reload the Docker daemon configuration.
apt-get update && apt-get install -y \
  nvidia-docker2=${NVIDIA_DOCKER_VERSION} \
  nvidia-container-runtime=${NVIDIA_RUNTIME_VERSION}

# Test the Nvidia Docker installation after making sure that the service is running and that Nvidia drivers are found.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
if [ -e /proc/driver/nvidia/version ]; then
  docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
fi

################################################################################

# Install Terminator terminal.
# https://gnometerminator.blogspot.com/

# Install the latest version of Terminator from the Ubuntu repositories.
apt-get update && apt-get install -y \
  terminator

# Prevent the Terminator installation to replace the default Ubuntu terminal.
update-alternatives --set x-terminal-emulator /usr/bin/gnome-terminal.wrapper
