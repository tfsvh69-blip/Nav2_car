# carcar_description

本包维护四轮麦克纳姆巡检小车的 URDF、静态 TF 和 RViz2 模型查看入口。

坐标遵循 ROS 约定：X 指向车头、Y 指向车体左侧、Z 向上。`base_footprint` 位于四轮
接地点构成区域的地面中心，`base_link` 位于四个轮心构成矩形的中心并与轮轴等高。

当前已确认参数：轮径 `60 mm`、轮宽 `31 mm`、前后轮心距 `120 mm`、左右轮心距
`185 mm`。底盘外壳只做简化显示，雷达、摄像头和 IMU 位姿仍是临时值，不能用于导航
验收。当前 `camera_link/camera_optical_frame` 只是未接相机时的模型占位；接入 D435 后，
本包只维护 `base_link -> camera_link` 实测安装外参，D435 内部传感器和 optical TF 由
官方 `realsense-ros` 驱动发布，避免重复 TF。

查看模型：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_description view_model.launch.py
```

该入口只启动机器人描述、零位关节状态和 RViz2，不打开串口或启动电机。

当前视觉模型使用银色简化底盘和白色麦克纳姆轮。轮子视觉 STL 来自 Apache-2.0
许可的 MOGI-ROS 开源模型，并按实物缩放至直径 `60 mm`、宽 `31 mm`；详细来源和固定
版本见 `third_party/mogi_ros/NOTICE.md`。碰撞几何仍使用轻量圆柱，外观不参与运动学计算。
