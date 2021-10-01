#!/bin/bash

# This script restarts a Docker container instance with a name based on [docker-project].
#
# Usage: bash RESTART-DOCKER-CONTAINER.bash [docker-project]
#
# [docker-project]: Used to select the Docker container to restart. Default value is '$USER'.

#################################################################################

# Set the Docker container name from the [docker-project] argument.
# If no [docker-project] is given, use the current user name as the Docker project name.
DOCKER_PROJECT=$1
if [ -z "${DOCKER_PROJECT}" ]; then
  DOCKER_PROJECT=${USER}
fi
DOCKER_CONTAINER="${DOCKER_PROJECT}_o2ac-ur_1"
echo "$0: DOCKER_PROJECT=${DOCKER_PROJECT}"
echo "$0: DOCKER_CONTAINER=${DOCKER_CONTAINER}"

################################################################################

# Display the Docker container status.
DOCKER_STATUS=`docker ps -a -f "Name=${DOCKER_CONTAINER}" --format "{{.Status}}" | cut -d ' ' -f 1`
echo "$0: DOCKER_STATUS=${DOCKER_STATUS}"

################################################################################

# Restart the Docker container.

# If the container is running, then stop and restart it.
if [ $DOCKER_STATUS == "Up" ]; then
  echo "Stopping ${DOCKER_CONTAINER}..."
  docker stop ${DOCKER_CONTAINER}
  echo "${DOCKER_CONTAINER} stopped."
fi

echo "Starting ${DOCKER_CONTAINER}..."
docker start ${DOCKER_CONTAINER}
echo "${DOCKER_CONTAINER} started."
