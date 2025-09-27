#!/bin/bash
# set -x
# set -e
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${PROJECT_DIR}/scripts/common_func.sh
echo_G  "[START] Vins-Fusion "

VINS_OUTPUT_DIR="${PROJECT_DIR}/src/global_interface/config/vins_fusion/output"
if [ ! -d "${VINS_OUTPUT_DIR}" ]; then
    mkdir ${VINS_OUTPUT_DIR}
else
    echo "Vins-Fusion output dir is already exit."
fi

source ${PROJECT_DIR}/devel/setup.bash;roslaunch vins swarm_d430_no_rviz.launch  && sleep 1;