#!/bin/bash

set -e

cd /TrailblazerML

rosdep install --from-paths src --ignore-src -r -y

SETUP_BASH="install/setup.bash"

if [ ! -d "install" ]; then
    echo "Workspace not built. Building with colcon"
    colcon build
else
    echo "Workspace already built"
fi

if [ -f "$SETUP_BASH" ]; then
    # shellcheck disable=SC1090
    source install/setup.bash
else
    echo "$SETUP_BASH does not exist. Please build your workspace first."
    exit 1
fi
cd /TrailblazerML
terminator
exec "$@"