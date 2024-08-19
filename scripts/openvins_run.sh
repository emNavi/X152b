#!/bin/bash
echo "[START] OpenVINS"

roslaunch ov_msckf subscribe.launch config:=rs_d430

# rviz -d src/open_vins/ov_msckf/launch/display.rviz

