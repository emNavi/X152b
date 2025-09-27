#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh
echo_G  "[START] Ego_Planner_Swarm_V1 "

drone_type=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_type)
drone_id=$(python3 ${PROJECT_DIR}/scripts/find_config.py drone_id)
cx=$(python3 ${PROJECT_DIR}/scripts/find_config.py cx)
cy=$(python3 ${PROJECT_DIR}/scripts/find_config.py cy)
fx=$(python3 ${PROJECT_DIR}/scripts/find_config.py fx)
fy=$(python3 ${PROJECT_DIR}/scripts/find_config.py fy)
max_vel=$(python3 ${PROJECT_DIR}/scripts/find_config.py max_vel)
max_acc=$(python3 ${PROJECT_DIR}/scripts/find_config.py max_acc)
ego_swarm_flage=$(python3 ${PROJECT_DIR}/scripts/find_config.py ego_swarm_flage)
flight_type=$(python3 ${PROJECT_DIR}/scripts/find_config.py flight_type)
echo_G  "Drone $drone_type - $drone_id run in max_vel $max_vel: max_acc: $max_acc"

#clean local_map
rosparam set clean_localmap true

if [ $? -eq 0 ] 
then
    source ${PROJECT_DIR}/devel/setup.bash;roslaunch ego_planner swarm_all_in_one.launch \
    drone_type:=$drone_type drone_id:=$drone_id cx:=$cx cy:=$cy fx:=$fx fy:=$fy flight_type:=$flight_type max_vel:=$max_vel max_acc:=$max_acc ego_swarm_flage:=$ego_swarm_flage && sleep 1;

else
    echo error
fi