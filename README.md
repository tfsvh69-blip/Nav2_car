# carcar 四轮麦克纳姆导航巡检小车

面向 ROS 2 Humble 的四轮麦克纳姆室内巡检小车。目标是多点导航巡检、RealSense D435
视觉自主回充，以及手机 Web 遥控和状态查看。当前进度与下一步以
[项目现状与路线](docs/项目现状与路线.md)为准。

## 软件分层

```text
硬件串口 / Rosmaster 控制板
        ↓
rosmaster_vendor       原厂 Python 驱动（第三方代码，仅封装安装）
        ↓
carcar_base            cmd_vel、电机、IMU、编码器、里程计、电池
        ↓
carcar_description     URDF/Xacro 与 base_link/传感器静态 TF
        ↓
carcar_lidar           RPLIDAR A1 的 Humble 驱动编排与 RViz 测试
        ↓
carcar_navigation      SLAM、定位、Nav2 参数与启动入口
        ↓
carcar_algorithms      后续感知/规划算法
        ↓
carcar_inspection      后续巡检任务、状态机与业务接口

carcar_bringup         只负责编排各层，不承载业务代码
```

项目文档入口见 [文档索引](docs/文档索引.md)。其中包含架构设计、硬件台账、完整测试记录和上车前检查说明。

M1～M4 轮位、机械方向、逐路停车和独立编码反馈已经确认，旧[电机测试](docs/电机测试.md)
只作故障回归，不需要继续重复逐轮实验。当前优先测量雷达、IMU、D435 安装 TF 和整车外廓。

RPLIDAR A1 在当前 Jetson/Humble 上的独立测试请按[激光雷达 Humble 迁移与测试](docs/激光雷达Humble迁移与测试.md)执行。V1.0 的 Jazzy 雷达入口继续保留，不作为 Humble 的启动命令。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch carcar_bringup robot.launch.py
```

如果系统尚未初始化 rosdep，请先执行 `sudo rosdep init`（仅首次）和 `rosdep update`。导航阶段需要安装 `nav2_bringup` 与 `slam_toolbox`；依赖声明会让 rosdep 自动处理它们。

先架空轮子测试：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
ros2 topic echo /imu/data_raw
ros2 topic echo /wheel/odometry
```

停止发送速度指令后，驱动节点会在 0.5 秒内触发看门狗并停车。

## 导航阶段入口

建图、保存地图后导航的命令已预留：

```bash
# 终端 1：机器人本体
ros2 launch carcar_bringup robot.launch.py

# 终端 2：建图
ros2 launch carcar_navigation slam.launch.py

# 保存地图（示例）
ros2 run nav2_map_server map_saver_cli -f maps/site

# 使用已有地图定位与导航
ros2 launch carcar_navigation navigation.launch.py map:=/absolute/path/to/site.yaml
```

轮径、轮宽、轴距和轮距已经写入描述；底盘最大外廓、雷达、IMU、D435 安装位仍是临时
占位值。完成实测并更新 `carcar_description/urdf/carcar.urdf.xacro` 和 Nav2 footprint
之前，不进行导航验收。
