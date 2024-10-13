#!/bin/bash
echo "[START] one_shot script with X152-B "
# 通过本脚本文件路径来获取 x152b 项目文件根目录
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
if [ $? -eq 0 ] 
then
    echo "[Drone id]: $drone_id"
    echo "start global interface"
    gnome-terminal -- bash -c "source devel/setup.bash;roslaunch global_interface driver_start.launch drone_id:=$drone_id" && sleep 1;
    sleep 3;

    # # 开启视觉里程计 VINS
    # gnome-terminal -- bash -c "source devel/setup.bash;roslaunch vins swarm_d430_no_rviz.launch"  && sleep 1;

    # # 开启规控算法 Ego # TODO(Derkai): 需要条件编译,不然就和ego-planner的包名重复导致无法通过编译检查
    # bash ./ipc_run.sh && sleep 1;

else
    echo "[error]Not found drone_id,please_check params_file"
    exit 
fi
echo "if you want to close this script, you need use : ./S_kill_one_shot.sh"
echo "if you want to connect foxglove, you need use this address : ws://<drone_ip>:8765"
