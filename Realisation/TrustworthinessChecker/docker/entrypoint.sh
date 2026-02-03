#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/humble/setup.bash
# echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /opt/setup.bash ]
then
  source /opt/setup.bash
  # echo "Sourced TurtleBot4 base workspace"
fi

# Docker group should already exist from Dockerfile, but ensure user is in it
if [ -S /var/run/docker.sock ]; then
    DOCKER_GID=$(stat -c "%g" /var/run/docker.sock)
    sudo groupmod -g $DOCKER_GID docker 2>/dev/null || true
    sudo usermod -aG docker $(id -nu) 2>/dev/null || true
fi

# if [ "$EUID" -gt 0 ]; then
#   sudo chown -R ${UID}:${UID} ~/.ros
#   sudo chown -R ${UID}:${UID} ~/.ignition
# fi

# Execute the command passed into this entrypoint
exec "$@"
