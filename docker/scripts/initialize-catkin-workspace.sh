#!/bin/bash

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Source the updated ROS environment.
source /opt/ros/noetic/setup.bash

################################################################################

# Initialize and build the Catkin workspace.
cd /root/catkin_ws/ && catkin_make

# Source the Catkin workspace.
source /root/catkin_ws/devel/setup.bash
