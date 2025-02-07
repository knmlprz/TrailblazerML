#!/bin/bash

set -e

cd /TrailblazerML/scripts/
sh dowload_rover_stl.sh
cd /TrailblazerML

SETUP_BASH="install/setup.bash"

if [ ! -d "install" ]; then
    echo "Workspace not built. Building with colcon"
    colcon build --symlink-install
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
cd /
mkdir -p ~/.gazebo/models/gazebo_viz
cp -r /TrailblazerML/src/gazebo_viz/meshes ~/.gazebo/models/gazebo_viz
exec "$@"

