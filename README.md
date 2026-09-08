# carcar 四轮麦克纳姆导航巡检小车

2026-09-07 最新修订：用户已决定建模宽统一为 216 mm，原 213 mm 差异不再作为待办。
IMU 位置暂估为相对 `base_footprint=(-40,-15,75) mm`，轴向待验证。当前操作请以
[操作手册](docs/操作手册.md) 为准。

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

M1～M4 的历史轮位和编码反馈结果已归档。整车 description 已接入实车尺寸和 STL。当前不启动正式 bringup；先按
[操作手册](docs/操作手册.md) 完成 MOTOR-007。

RPLIDAR A1 在当前 Jetson/Humble 上已能稳定发布 `/scan`，详情见[硬件台账](docs/硬件台账.md)；历史迁移记录已归档。

## 快速开始

下列是依赖安装和离线构建，不启动硬件：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果系统尚未初始化 rosdep，请先执行 `sudo rosdep init`（仅首次）和 `rosdep update`。导航阶段需要安装 `nav2_bringup` 与 `slam_toolbox`；依赖声明会让 rosdep 自动处理它们。

硬件首启不使用通用 `/cmd_vel` 单次发布；请先完成 MOTOR-007 的断电接线
修正和架空短脉冲验收。正式驱动看门狗已改为 `0.3 s`。

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

轮径、轮宽、轴距、轮距、整车外廓、顶板、雷达位置和 RealSense 左红外镜头位置已经写入
描述。整车宽按用户决定统一为 `216 mm`；雷达平面方向已通过，相机倒装已按
`roll=pi` 写入模型。IMU 完整轴向仍待验证。完成这些确认并更新 Nav2 footprint 前，
不进行导航验收。
