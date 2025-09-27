#!/bin/bash
# 通过本脚本文件路径来获取 X152b 项目文件根目录、飞机类型、飞机编号、起飞高度、gcs_udp_ip
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh

echo_G "[START] Init UAV with X152b"
drone_type=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_type)
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
takeoff_height=$(python3 ${PROJECT_DIR}/scripts/find_config.py takeoff_height)
gcs_udp_ip=$(python3 ${PROJECT_DIR}/scripts/find_config.py gcs_udp_ip)

if [ $? -eq 0 ] 
then
    echo_G "[Drone]: $drone_type - $drone_id"
    echo_G "start global interface"
    source ${PROJECT_DIR}/devel/setup.bash;roslaunch global_interface driver_start.launch drone_type:=$drone_type drone_id:=$drone_id takeoff_height:=$takeoff_height gcs_udp_ip:=$gcs_udp_ip && sleep 1;
    sleep 3;

else
    echo_R "[ERROR] please_check params_file"
    exit 
fi