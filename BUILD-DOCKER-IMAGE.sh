#!/bin/bash
# Generates a docker image with the relevant settings for the DOCKER_PROJECT.
# Context-sensitive behaviour: If the <user> parameter "gitlab-ci" is used, 
# the script builds the image without trying to download it.
#
# Usage: ./BUILD-DOCKER-IMAGE.bash <optional: user>
#
# @param <user> [optional parameter] for docker container naming during spin-up and to set different behavior.
#                  Default value: $USER - the image is pulled from repo, or built as fallback.
#                  If <user> is "gitlab-ci" the image is directly build from scratch - as if done in gitlab-ci.
################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
DOCKER_PROJECT=$1
if [ -z "${DOCKER_PROJECT}" ]; then
  DOCKER_PROJECT=${USER}
fi
DOCKER_CONTAINER="${DOCKER_PROJECT}_o2ac-ur_1"
echo "$0: DOCKER_PROJECT=${DOCKER_PROJECT}"
echo "$0: DOCKER_CONTAINER=${DOCKER_CONTAINER}"

# Stop and remove the Docker container.
EXISTING_DOCKER_CONTAINER_ID=`docker ps -aq -f name=${DOCKER_CONTAINER}`
if [ ! -z "${EXISTING_DOCKER_CONTAINER_ID}" ]; then
  echo "Stop the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker stop ${EXISTING_DOCKER_CONTAINER_ID}
  echo "Remove the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker rm ${EXISTING_DOCKER_CONTAINER_ID}
fi

################################################################################

# Inject Dockerfile's commit hash into docker container implicitly via docker-compose.yml build-arg.
# (Setting ENV variable invalidates after script terminates)
export DOCKERFILE_COMMIT_SHORT_SHA="$(git log -n 1 --pretty=format:%h docker/Dockerfile)"

# Login to gitlab docker registry to authenticate
# echo ""
# echo "Login to registry.gitlab.com... Enter your username and password:"
# docker login registry.gitlab.com

# If the script runs in CI mode, image builds directly.
# Otherwise try to pull it from the server. If this fails, it has to be build locally.
# if [[ "$DOCKER_PROJECT" == "gitlab-ci" ]]; then
#   docker-compose -p ${DOCKER_PROJECT} -f ./docker/docker-compose.yml build
# else
#   # regular developer use case
#   docker-compose -p ${DOCKER_PROJECT} -f ./docker/docker-compose.yml pull
#   if [[ "$?" -ne "0" ]]; then
#     docker-compose -p ${DOCKER_PROJECT} -f ./docker/docker-compose.yml build
#   fi
# fi

docker-compose -p ${DOCKER_PROJECT} -f ./docker/docker-compose.yml build
