#!/bin/bash
echo "<Takeoff>"

# 通过本脚本文件路径来获取 X152b 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
if [ $? -eq 0 ] 
then 
    rostopic pub /emnavi_cmd/takeoff std_msgs/String "
    data: 'drone_$drone_id'
    "  --once
else
    echo error
fi

echo "Drone $drone_id Takeoff"
# 发送该命令后，在启动脚本能够重置飞行状态机状态，能再次起飞；起飞完成后，将关闭重置飞行状态的功能。
# rosparam set /drone_fsm_reset true