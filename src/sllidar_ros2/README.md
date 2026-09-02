# SLAMTEC 激光雷达 ROS 2 驱动包

这是 SLAMTEC 激光雷达的 ROS 2 节点。本目录为上游 `sllidar_ros2` 驱动源码；本项目的统一启动和 RViz 配置位于 `src/inspection_bringup/`。

相关资料：

- [SLAMTEC 雷达 ROS Wiki](http://wiki.ros.org/rplidar)
- [SLAMTEC 雷达官网](http://www.slamtec.com/en/Lidar)
- [SLAMTEC 雷达 SDK](https://github.com/Slamtec/rplidar_sdk)
- [RPLIDAR 教程](https://github.com/robopeak/rplidar_ros/wiki)

## 支持的雷达型号

| 雷达型号 |
| --- |
| RPLIDAR A1 |
| RPLIDAR A2 |
| RPLIDAR A3 |
| RPLIDAR C1 |
| RPLIDAR S1 |
| RPLIDAR S2 |
| RPLIDAR S3 |
| RPLIDAR S2E |
| RPLIDAR T1 |

## ROS 2 环境

请先按照 [ROS 2 官方安装文档](https://docs.ros.org/en/rolling/Installation.html)安装对应发行版，并完成 [ROS 2 环境配置](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)。本项目使用 ROS 2 Jazzy，构建前执行：

```bash
source /opt/ros/jazzy/setup.bash
```

## 创建工作空间

一般 ROS 2 工作空间可按以下方式创建：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

本仓库已经将驱动放在 `src/sllidar_ros2/`，无需重复克隆。

## 编译与加载环境

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

在本项目中也可以直接运行：

```bash
./scripts/build.sh
```

如果系统找不到 `colcon`，请按照 [colcon 教程](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon)安装构建工具。

## 串口权限

驱动需要读取和写入雷达串口。推荐将当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

注销并重新登录后生效。临时测试可以执行：

```bash
sudo chmod a+rw /dev/ttyUSB0
```

不要使用 `sudo` 启动 ROS 节点或 RViz。也可以根据实际设备配置 udev 规则；本驱动提供 `scripts/create_udev_rules.sh` 作为参考。

## 启动驱动与 RViz

本项目中的 RPLIDAR A1 推荐使用统一入口：

```bash
./scripts/run_lidar.sh
```

直接调用上游 launch 文件时，各型号对应命令如下。

### RPLIDAR A1

```bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

### RPLIDAR A2M7

```bash
ros2 launch sllidar_ros2 view_sllidar_a2m7_launch.py
```

### RPLIDAR A2M8

```bash
ros2 launch sllidar_ros2 view_sllidar_a2m8_launch.py
```

### RPLIDAR A2M12

```bash
ros2 launch sllidar_ros2 view_sllidar_a2m12_launch.py
```

### RPLIDAR A3

```bash
ros2 launch sllidar_ros2 view_sllidar_a3_launch.py
```

### RPLIDAR C1

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```

### RPLIDAR S1

```bash
ros2 launch sllidar_ros2 view_sllidar_s1_launch.py
```

使用 TCP 连接 S1：

```bash
ros2 launch sllidar_ros2 view_sllidar_s1_tcp_launch.py
```

### RPLIDAR S2 与 S2E

```bash
ros2 launch sllidar_ros2 view_sllidar_s2_launch.py
ros2 launch sllidar_ros2 view_sllidar_s2e_launch.py
```

### RPLIDAR S3

```bash
ros2 launch sllidar_ros2 view_sllidar_s3_launch.py
```

### RPLIDAR T1

```bash
ros2 launch sllidar_ros2 view_sllidar_t1_launch.py
```

不同雷达型号使用的 `serial_baudrate` 可能不同，请以设备资料和对应 launch 文件为准。

## 雷达坐标系与数据检查

雷达通常发布 `sensor_msgs/msg/LaserScan` 类型的 `/scan`。检查发布频率和单帧数据：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

雷达坐标系必须通过 TF 发布。本项目测试入口默认发布 `base_link → laser` 静态变换；实际装车时应根据安装位置设置平移和旋转参数，并由机器人描述统一维护 TF。
