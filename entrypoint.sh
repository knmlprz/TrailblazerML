#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

SETUP_BASH="install/setup.bash"

if [ ! -d "install" ]; then
    echo "Workspace not built. Building with colcon"
    colcon build --symlink-install
else
    echo "Workspace already built"
fi

if [ -f "$SETUP_BASH" ]; then
    source "$SETUP_BASH"
else
    echo "$SETUP_BASH does not exist. Please build your workspace first."
    exit 1
fi

exec "$@"

