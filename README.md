# 巡检小车 ROS 2 工程

这是一个独立的 ROS 2 Jazzy 工作空间。当前阶段只接入 RPLIDAR A1，目标是验证雷达能否发布 `/scan`，并在 RViz2 中显示；后续可继续加入底盘、深度相机、机器人描述和 Nav2。

## 目录结构

```text
my_nav2_car/
├── src/
│   ├── inspection_bringup/   # 本项目统一启动、参数和 RViz 配置
│   └── sllidar_ros2/         # SLAMTEC 官方驱动
├── docs/                      # 硬件实测结论与长期维护记录
├── scripts/                  # 构建、设备检查和启动脚本
└── dependencies.repos        # 外部源码版本记录
```

所有构建产物都生成在本目录的 `build/`、`install/`、`log/` 中。

## 1. 插入雷达并检查设备

不要给不确定用途的红黑线直接接 12 V。先连接原装 USB 转接板，然后执行：

```bash
./scripts/check_lidar.sh
```

早期 RPLIDAR A1 通常显示为 `/dev/ttyUSB0`，波特率为 `115200`。如果没有串口，先检查 USB 接线和当前系统/容器是否获得 USB 设备访问权限。

本工程默认使用这颗 A1 固件支持的 `Express` 扫描模式（4 kHz），比 `Standard`（2 kHz）每圈点数更多。需要排障时，可在 launch 命令后传入 `scan_mode:=Standard`。

当前硬件已确认的稳定读数范围为 **0.10～3.00 m**。这是当前设备和测试条件下的实测稳定范围，不等同于驱动报告的 12 m 理论最大量程。详细信息、后续更新和可复制命令见[硬件实测记录](docs/硬件实测记录.md#常用命令速查)。

如果串口存在但当前用户没有权限，临时测试可执行：

```bash
sudo chmod a+rw /dev/ttyUSB0
```

长期使用建议配置 udev 规则或把用户加入系统的串口设备组，而不是每次修改权限。

## 2. 构建

```bash
./scripts/build.sh
```

## 3. 启动雷达和 RViz2

```bash
./scripts/run_lidar.sh
```

脚本默认使用 `/dev/ttyUSB0`。如果设备名不同：

```bash
./scripts/run_lidar.sh /dev/ttyUSB1
```

也可以直接使用 ROS 2 launch，并覆盖任意参数：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/ros"
ros2 launch inspection_bringup lidar_test.launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=115200 \
  use_rviz:=true
```

启动成功后，RViz2 中应出现一圈随障碍物变化的点。另开终端验证数据：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## 常见问题

- `cannot open /dev/ttyUSB0`：设备名不对、设备未透传到当前环境或串口权限不足。
- `Wrong body size`、无数据或通信超时：先确认型号；A1 用 `115200`，不要直接套用较新型号的高波特率。
- `scan mode ... is not supported` 或 `Can not start scan: 80008001`：扫描模式不受当前固件支持。本机已经确认支持 `Standard`、`Express`、`Boost` 和 `Stability`，不支持 `Sensitivity`。
- 电机转但没有扫描点：查看启动终端里的 health/通信错误，并确认 RViz Fixed Frame 为 `base_link`、LaserScan Topic 为 `/scan`，且存在 `base_link → laser` TF。
- RViz 报 Qt/display 错误：当前会话没有图形显示能力。可先传入 `use_rviz:=false`，用 `ros2 topic hz /scan` 验证雷达数据。
- 点云方向相反：启动时增加 `inverted:=true`。

## 后续扩展约定

- `inspection_bringup`：全车 launch 与运行参数。
- 后续建议新增 `inspection_description`：URDF/Xacro 和 TF。
- 后续建议新增 `inspection_base`：底盘通信、里程计与控制。
- 后续建议新增 `inspection_perception`：深度相机及感知节点。
- 后续建议新增 `inspection_navigation`：SLAM、定位和 Nav2 参数。

各硬件驱动保持为独立上游包，本项目只在 bringup 层组合它们，便于替换硬件和单独测试。
