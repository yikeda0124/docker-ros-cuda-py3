#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
if [ -z $ROS_PROJECT_NAME ]; then
  echo "Set ROS_PROJECT_NAME (e.g. 'export ROS_PROJECT_NAME=mytest')"
  exit 1
fi
PROJECT=$ROS_PROJECT_NAME
CONTAINER="${PROJECT}_ros_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Stop and remove the Docker container.
EXISTING_CONTAINER_ID=`docker ps -aq -f name=${CONTAINER}`
if [ ! -z "${EXISTING_CONTAINER_ID}" ]; then
  echo "The container name ${CONTAINER} is already in use" 1>&2
  echo ${EXISTING_CONTAINER_ID}
  exit 1
fi

################################################################################

# Build the Docker image with the Nvidia GL library.
echo "starting build"
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml build