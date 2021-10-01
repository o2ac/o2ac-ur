#!/bin/bash

# This script implements a mutex mechanism on a branch basis to create mutexes on a GitLab Runner '/data' folder.
# The purpose is to avoid write access by multiple runners on the same shared folder in the same branch.
# Before a process writes to the '/data' folder, a mutex should be created by calling this script with the paramter value 'createOrWait'.
# If the resource is already locked, the script waits and tries continously to lock the resource.
# After finishing the write job, the script has to be called again to release the resource lock with the parameter value 'release'.
# 
# Usage: bash lock-shared-resource.bash <task> <hash-value>
#
# <task>: Accepted value is 'createOrWait' or 'release'.
# <hash-value>: Some token value.

################################################################################

# Check if the user has provided the mandatory <task> argument.
if [[ "$#" -ne 2 && ( $1 == "createOrWait" || $1 == "release" ) ]]; then
  echo "Usage: bash lock-shared-resource.bash <createOrWait | release>"
  exit 1
fi

################################################################################

# Define resource to lock.
LOCK_FILE="/data/docker-image-$2-write-lock"

################################################################################

# Request a lock.
if [[ $1 == "createOrWait" ]]; then
  # If an other task has created a lock on that branch, wait until it is released.
  while [[ -f ${LOCK_FILE} ]]
  do
    echo "Waiting for free resource at '${LOCK_FILE}'."
    sleep 5
  done

  touch ${LOCK_FILE}
  echo "Created resource lock at '${LOCK_FILE}'."
  exit 0
fi

################################################################################

# Release the lock.
if [[ $1 == "release" ]]; then
  rm -f ${LOCK_FILE}
  echo "Released resource lock at '${LOCK_FILE}'."
  exit 0
fi
