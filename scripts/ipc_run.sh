#!/bin/bash
echo "[START] IPC"

# 启动建图算法 OCMAP
gnome-terminal -- bash -c "source ../devel/setup.bash;roslaunch local_ocmap_gen grid_map.launch" && sleep 1;

# 启动 IPC 规划控制算法
gnome-terminal -- bash -c "source ../devel/setup.bash;roslaunch ipc ipc.launch" && sleep 1;

# 如果仅进行仿真规控测试
# gnome-terminal -- bash -c "source devel/setup.bash;roslaunch test_interface simulator.launch" && sleep 1;
# gnome-terminal -- bash -c "source devel/setup.bash;roslaunch ipc ipc_avoid.launch" && sleep 1;