#!/bin/bash

################################################################################

# Start the D-Bus daemon.
service dbus start

################################################################################

# Start the avahi daemon (for resolving DNS name)
service avahi-daemon start

################################################################################

# Keep the Docker container running in the background.
# https://stackoverflow.com/questions/30209776/docker-container-will-automatically-stop-after-docker-run-d
tail -f /dev/null