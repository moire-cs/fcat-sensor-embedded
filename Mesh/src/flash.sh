#!/bin/bash

# Prompt for starting node ID, default to 1 if empty
read -p "Enter starting node ID (default 1): " node_id
node_id=${node_id:-1}

while true; do
    # Edit the 15th line of ../src/node-numbers.h
    sed -i '' "15s/.*/const uint8_t selfAddress_ = $node_id;/" ./src/node-numbers.h
    # Run platformio commands
    platformio run --target upload --target monitor --environment sensor_node --upload-port /dev/cu.usbserial-10 --monitor-port /dev/cu.usbserial-10

    # Wait for user to exit the process
    read -p "Want to flash again with node id $((node_id+1)) (y/n)? " answer

    if [[ $answer == "y" ]]; then
        # Increment node ID
        ((node_id++))
    else
        # Ask for custom node ID or exit
        read -p "Would you like to set a custom id? (0-255/n): " custom_id
        if [[ $custom_id =~ ^[0-9]+$ ]] && [ $custom_id -ge 0 ] && [ $custom_id -le 255 ]; then
            node_id=$custom_id
        else
            exit 0
        fi
    fi
done
