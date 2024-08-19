#!/bin/bash
echo "[START] one_shot script with X152-B "
# TODO(Derkai): 这里获取ID的方式可能会变更
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
drone_id=$(python3 ${PROJECT_DIR}/src/tools/find_config.py drone_id)
if [ $? -eq 0 ] 
then
    echo "[Drone id]: $drone_id"
    echo "start global interface"
    gnome-terminal -- bash -c "source devel/setup.bash;roslaunch global_interface driver_start.launch drone_id:=$drone_id" && sleep 1;
    sleep 3;

    # 开启视觉里程计 VINS
    bash ./vins_fusion_run.sh && sleep 1;

    # 开启规控算法 IPC # TODO(Derkai): 需要条件编译,不然就和ego-planner的包名重复导致无法通过编译检查
    bash ./ipc_run.sh && sleep 1;

else
    echo "[error]Not found drone_id,please_check params_file"
    exit 
fi
echo "if you want to close this script, you need use : ./S_kill_one_shot.sh"
echo "if you want to connect foxglove, you need use this address : ws://<drone_ip>:8765"
