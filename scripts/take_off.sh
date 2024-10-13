#!/bin/bash
echo "<Takeoff>"


drone_id=$(python3 ./scripts/find_config.py drone_id)
if [ $? -eq 0 ] 
then
    rostopic pub /swarm_takeoff std_msgs/Float32MultiArray "
    layout: 
        dim: 
            - 
                label: ''
                size: 0
                stride: 0
        data_offset: 0
    data: [1, $drone_id]
    "  --once
else
    echo error
fi

echo "Drone $drone_id Takeoff"
