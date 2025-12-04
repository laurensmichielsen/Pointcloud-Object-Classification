#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "No input MCAP file provided"
    echo "Usage: $0 <input.mcap>"
    exit 1
fi

input_mcap=$1
# Get directory of input file
dir=$(dirname "$input_mcap")
# Extract filename without extension
filename=$(basename "$input_mcap" .mcap)
output_db3="$dir/${filename}_filtered"

# Start recording in background
ros2 bag record /rslidar_points -s sqlite3 "--qos-profile-overrides-path" "data/utils/qos_bag_override.yaml" -o "$output_db3" &
record_pid=$!

# Wait a moment to ensure recording has started
sleep 2

# Play the bag
ros2 bag play "$input_mcap" "--qos-profile-overrides-path" "data/utils/qos_bag_override.yaml"

# Wait a moment to ensure all messages are recorded
sleep 2

# Kill the recording process
kill $record_pid

echo "Conversion complete. Output saved as: ${output_db3}"
