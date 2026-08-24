#!/bin/bash
# Script to save SLAM map after mapping session

if [ -z "$1" ]; then
    MAP_NAME="kvn_rover_map_$(date +%Y%m%d_%H%M%S)"
else
    MAP_NAME="$1"
fi

SAVE_DIR="$HOME/ros_kvn_ws/maps"
mkdir -p "$SAVE_DIR"

echo "Saving map as: $MAP_NAME"
echo "Target directory: $SAVE_DIR"

# Use nav2's map_saver_server service
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: /map, map_url: $SAVE_DIR/$MAP_NAME, image_format: pgm, free_thresh: 0.15, occupied_thresh: 0.65}"

if [ $? -eq 0 ]; then
    echo "✓ Map saved successfully!"
    echo "  YAML: $SAVE_DIR/$MAP_NAME.yaml"
    echo "  PGM:  $SAVE_DIR/$MAP_NAME.pgm"
else
    echo "✗ Failed to save map"
    exit 1
fi
