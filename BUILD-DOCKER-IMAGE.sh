#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT=$1
if [ -z "${PROJECT}" ]; then
  PROJECT=${USER}
fi
CONTAINER="${PROJECT}_o2ac-ur_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Stop and remove the Docker container.
EXISTING_CONTAINER_ID=`docker ps -aq -f name=${CONTAINER}`
if [ ! -z "${EXISTING_CONTAINER_ID}" ]; then
  echo "Stop the container ${CONTAINER} with ID: ${EXISTING_CONTAINER_ID}."
  docker stop ${EXISTING_CONTAINER_ID}
  echo "Remove the container ${CONTAINER} with ID: ${EXISTING_CONTAINER_ID}."
  docker rm ${EXISTING_CONTAINER_ID}
fi

################################################################################

# Build the Docker image.
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml build
