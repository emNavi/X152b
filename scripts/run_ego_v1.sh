#!/bin/bash
echo "<Ego_planner>"

max_vel=1.0
max_acc=1.0
drone_id=$(python3 ./scripts/find_config.py drone_id)
# <!-- 1: use 2D Nav Goal to select goal  -->
# <!-- 2: use global waypoints below  -->
# <!-- 4: use REMOTE_TARGET  -->
# <!-- 5: use REMOTE_START  -->
flight_type=2
echo "Drone $drone_id  autonomous run in max_vel $max_vel max_acc:=$max_acc"

cx=$(python3 ./scripts/find_config.py cx)
cy=$(python3 ./scripts/find_config.py cy)
fx=$(python3 ./scripts/find_config.py fx)
fy=$(python3 ./scripts/find_config.py fy)
if [ $? -eq 0 ] 
then
    gnome-terminal -- bash -c "source devel/setup.bash;roslaunch ego_planner swarm_all_in_one.launch \
    drone_id:=$drone_id cx:=$cx cy:=$cy fx:=$fx fy:=$fy flight_type:=$flight_type max_vel:=$max_vel max_acc:=$max_acc" && sleep 1;
else
    echo error
fi