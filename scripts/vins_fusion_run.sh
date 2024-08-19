#!/bin/bash
echo "[START] VINS "

gnome-terminal -- bash -c "source ../devel/setup.bash;roslaunch vins swarm_d430_no_rviz.launch"