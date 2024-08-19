# Control driver
本包提供了无人机的单机、多机运动控制模块

## requirement
```bash
sudo apt install ros-noetic-mavros-extras
```
## ego_planner 实机使用
```bash
roslaunch single_offb_pkg swarm_single.launch drone_id:=EGO_ID
```

### 逻辑说明：
用ID号区分每一架飞机，并使用状态机进行状态条件切换
- INIT_PARAM = 1,
- TAKEOFF,
- HOVER,
- RUNNING,
- LANDING