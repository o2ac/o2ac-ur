#!/bin/bash

# Script for updating the Docker image on the Gitlab registry. (Only used by maintainers)

LOCAL_DOCKER_IMAGE="${USER}_o2ac-ur"  # Assuming the default image name

CI_REGISTRY_IMAGE=o2ac
DOCKER_FILE_HASH=`git log -n 1 --no-merges --pretty=format:%h ./docker/Dockerfile`
# Load the Docker image from the persistent data volume in the GitLab CI Docker executor.
# docker load -i /data/docker-image-${DOCKER_FILE_HASH}.tar
# Tag the latest successfully tested Docker image with an explicit commit hash for tracing.
docker tag ${LOCAL_DOCKER_IMAGE} registry.gitlab.com/${CI_REGISTRY_IMAGE}/o2ac-dev:${DOCKER_FILE_HASH}
docker tag ${LOCAL_DOCKER_IMAGE} registry.gitlab.com/${CI_REGISTRY_IMAGE}/o2ac-dev:latest
# Push the Docker image as 'latest' for new default, and as '${DOCKER_FILE_HASH}' for backup, into the GitLab registry.
# https://gitlab.com/help/user/packages/container_registry/index.md#build-and-push-images-using-gitlab-cicd
docker login registry.gitlab.com
docker push registry.gitlab.com/${CI_REGISTRY_IMAGE}/o2ac-dev:${DOCKER_FILE_HASH}
docker push registry.gitlab.com/${CI_REGISTRY_IMAGE}/o2ac-dev:latest
  