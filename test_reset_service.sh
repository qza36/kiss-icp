#!/bin/bash
# Test script to call the reset service
# Usage: ./test_reset_service.sh

echo "Testing KISS-ICP reset service..."

# Check if ROS 2 environment is sourced
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS 2 not found. Please source your ROS 2 installation."
    exit 1
fi

# Try to call the service
echo "Calling reset service..."
ros2 service call /kiss/reset std_srvs/srv/Empty

if [ $? -eq 0 ]; then
    echo "Reset service called successfully!"
else
    echo "Failed to call reset service. Make sure the kiss_icp_node is running."
fi