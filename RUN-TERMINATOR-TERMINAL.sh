#!/bin/bash

if [ -z $ROS_PROJECT_NAME ]; then
  echo "Set ROS_PROJECT_NAME (e.g. 'export ROS_PROJECT_NAME=mytest')"
  exit 1
fi
export PROJECT=$ROS_PROJECT_NAME
echo "$0: PROJECT=${PROJECT}"

################################################################################

# Make a backup of the user's original Terminator configuration.
mkdir -p ~/.config/terminator/
if [ ! -f ~/.config/terminator/config.backup ]; then
  cp ~/.config/terminator/config ~/.config/terminator/config.backup
fi

# Update the user's current Terminator configuration with the project's one.
cp ./terminator/config ~/.config/terminator/config

################################################################################

# Run Terminator with the project's custom layout.
terminator -m -l ros-term-yikeda &
sleep 1