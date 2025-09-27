#!/bin/bash
# 通过本脚本文件路径来获取 X152b 项目文件根目录、飞机类型、飞机编号、起飞高度、gcs_udp_ip
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh

echo_G "[START] Kill UAV with X152b"
rosrun emnv_ctl_bridge reboot_px4_system.py && sleep 1;

nodes=(
/rosout
/ctrl_bridge
/drone_1_ego_planner_node
/drone_1_traj_server
/local_map
/mavros
)

for n in "${nodes[@]}"; do
    echo "Killing $n ..."
    rosnode kill "$n"
done