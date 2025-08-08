#!/usr/bin/env bash

echo "Uruchomiles init.bash"

if [ -n "$ROS_DISTRO" ]; then
    echo "🚀 Aktualna dystrybucja ROS to: $ROS_DISTRO"
else
    echo "⚠️  ROS nie jest obecnie skonfigurowany w tym środowisku."
fi
