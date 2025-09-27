## 栅格建图算法
建图代码摘选自 [ego-planner-swarm 的 plan_env 模块](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/master/src/planner/plan_env) ，修改了点云坐标系错误的问题，使用朴素的投射方法进行栅格地图的实时构建，使用Realsense D435i，在Intel i7-9750 CPU上运行建图，可达到10Hz，CPU消耗占比40%

### 修改配置文件 (grid_map.launch)
```xml
# 按实际情况设置传感器话题名称（里程计话题、深度图话题）
    <arg name="odom_topic" value="/vins_fusion/odometry"/>
    <arg name="depth_topic" value="/camera/depth/image_rect_raw"/>

# 按实际情况修改深度相机内参
    <!-- intrinsic params of the depth camera -->
    <arg name="cx" value="321.91"/>
    <arg name="cy" value="241.553"/>
    <arg name="fx" value="386.098571777344"/>
    <arg name="fy" value="386.098571777344"/>
```

### 使用
```bash
#  开启D435i相机
roslaunch realsense2_camera rs_camera_d435i.launch

# 开启视觉里程计 VINS
roslaunch vins swarm_d435_no_rviz.launch

# 开启建图算法
roslaunch local_ocmap_gen grid_map.launch
```

![](docs/ocmap.png)