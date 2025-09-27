#!/bin/bash
echo "<Land>"

# 通过本脚本文件路径来获取 X152b 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
if [ $? -eq 0 ] 
then
    rostopic pub /emnavi_cmd/land std_msgs/String "
    data: 'drone_$drone_id'
    "  --once
else
    echo error
fi

echo "Drone $drone_id land"