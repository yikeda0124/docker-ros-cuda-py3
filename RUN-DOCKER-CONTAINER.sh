#!/bin/bash

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
if [ -z $ROS_PROJECT_NAME ]; then
  PROJECT=$ROS_PROJECT_NAME
  echo "Set ROS_PROJECT_NAME (e.g. 'export ROS_PROJECT_NAME=mytest')"
  exit 1
fi
PROJECT=$ROS_PROJECT_NAME
CONTAINER="${PROJECT}_ros_1"
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"

# Run the Docker container in the background.
# Any changes made to './docker/docker-compose.yml' will recreate and overwrite the container.
docker-compose -p ${PROJECT} -f ./docker/docker-compose.yml up -d

# Display GUI through X Server by granting full access to any external client.
xhost +

# Enter the Docker container with a Bash shell (with or without a custom 'roslaunch' command).
case "$2" in
  ( "" )
  docker exec -i -t ${CONTAINER} bash
  ;;
  ( * )
  echo "Failed to enter the Docker container '${CONTAINER}': '$2' is not a valid argument value."
  ;;
esac
