# 激光雷达 ROS 2 Humble 迁移与 RViz 测试

本文说明如何在当前 NVIDIA Jetson（Ubuntu 22.04、ROS 2 Humble）上构建、启动和验证 RPLIDAR A1。V1.0 中基于 Ubuntu 24.04、ROS 2 Jazzy 的 `inspection_bringup`、`scripts/build.sh` 和 `scripts/run_lidar.sh` 保留为历史测试资产，不用于当前 Humble 测试。

当前结论（2026-09-06）：Humble 驱动、`Express` 扫描和 `/scan` 连续发布已经可用，用户
确认当前稳定有效距离为 0.10～3.00 m。无需重复迁移测试；本专题以后只用于故障回归，
当前主任务是测量 `base_link -> laser_frame` 实际位姿并在 RViz 核对前/左/右方向。

## 一、迁移结论与文件入口

- 厂商驱动 `src/sllidar_ros2` 未改写，继续使用 V1.0 中已验证的固定版本。
- 新增 `carcar_lidar` Humble 集成包。
- 正式入口 `lidar.launch.py` 只启动雷达驱动，不发布临时 TF、不启动 RViz。
- 测试入口 `lidar_rviz_test.launch.py` 启动雷达、测试用静态 TF 和 RViz2。
- 默认 LaserScan 话题为 `/scan`，消息类型为 `sensor_msgs/msg/LaserScan`。
- 默认雷达帧为 `laser_frame`，与当前 `carcar_description` 一致。
- 默认扫描模式为已由设备枚举支持的 `Express`；若启动失败，先用 `Standard` 排查。

当前主机的串口身份如下：

| 硬件 | USB ID | 当前端口 | 稳定入口 |
|---|---|---|---|
| Rosmaster 底层板 | `1a86:7523`（CH340） | `/dev/ttyUSB0` | `/dev/myserial` |
| RPLIDAR A1 转接板 | `10c4:ea60`（CP2102） | `/dev/ttyUSB1` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |

不要把 `/dev/ttyUSB0` 或 `/dev/myserial` 传给雷达驱动。USB 插拔后 `ttyUSB` 编号可能变化，应优先使用 `by-id` 路径。

## 二、安全条件

1. 雷达使用原装 USB 转接板供电，不向用途不明的线缆额外加电。
2. 同一串口同时只能由一个进程打开；启动前必须检查占用。
3. 不使用 `sudo` 启动 ROS 节点或 RViz。
4. 雷达旋转时不要触碰转子；出现异响、卡滞、冒烟或过热时立即拔掉雷达 USB。
5. 本测试不启动 `carcar_base`，不会向 M1～M4 输出电机命令。

## 三、安装依赖

以下命令可直接复制。`rosdep init` 只在系统从未初始化时执行一次：

```bash
cd /home/jetson/luhao/my_nav_carcar
sudo rosdep init 2>/dev/null || true
rosdep update
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rviz2 \
  ros-humble-tf2-ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 四、识别端口与检查占用

```bash
cd /home/jetson/luhao/my_nav_carcar
./scripts/check_lidar_humble.sh
```

预期同时看到：

- `[雷达 CP210x] /dev/ttyUSB1`；
- `[底层板 CH340，请勿作为雷达端口] /dev/ttyUSB0`；
- 雷达端口状态为“未发现占用进程”。

也可以人工复核：

```bash
udevadm info --query=property --name=/dev/ttyUSB1 | \
  grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|DEVLINKS)='
fuser -v /dev/ttyUSB1 2>/dev/null || true
groups
```

若用户不属于 `dialout`，执行以下命令后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## 五、Humble 构建

本机登录环境会自动加入 Archiconda Python 3.7，而 ROS 2 Humble 使用 Ubuntu 系统 Python 3.10。必须使用下列脚本强制选择 `/usr/bin/python3`：

```bash
cd /home/jetson/luhao/my_nav_carcar
./scripts/build_lidar_humble.sh
```

预期 `sllidar_ros2` 和 `carcar_lidar` 均显示 `Finished`。如果需要不用脚本手动构建，完整命令为：

```bash
cd /home/jetson/luhao/my_nav_carcar
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to carcar_lidar \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 六、先做无 RViz 数据测试

终端 1 启动雷达，避免图形界面影响首次诊断：

```bash
cd /home/jetson/luhao/my_nav_carcar
./scripts/run_lidar_humble.sh \
  /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  false
```

终端 2 检查节点、参数和数据：

```bash
cd /home/jetson/luhao/my_nav_carcar
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic info /scan --verbose
ros2 topic hz /scan
```

让 `ros2 topic hz /scan` 至少运行 10 秒，按 `Ctrl+C` 结束统计。再分别执行：

```bash
ros2 topic echo /scan --once \
  --field header.frame_id
ros2 topic echo /scan --once \
  --field range_min
ros2 topic echo /scan --once \
  --field range_max
ros2 param get /sllidar_node serial_port
ros2 param get /sllidar_node serial_baudrate
ros2 param get /sllidar_node scan_mode
ros2 param get /sllidar_node frame_id
ros2 run tf2_ros tf2_echo base_link laser_frame
```

预期结果：

- `/sllidar_node` 存在；
- `/scan` 类型为 `sensor_msgs/msg/LaserScan`；
- `header.frame_id` 为 `laser_frame`；
- `/scan` 持续发布，没有反复掉线；
- `tf2_echo` 能看到 `base_link -> laser_frame` 的测试用静态变换。

如果 `Express` 报“不支持扫描模式”，停止终端 1 后改用已知兼容的 `Standard`：

```bash
cd /home/jetson/luhao/my_nav_carcar
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_lidar lidar_rviz_test.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  serial_baudrate:=115200 \
  scan_mode:=Standard \
  use_rviz:=false
```

## 七、RViz 数据测试

确认第六节持续有 `/scan` 后，先在终端 1 按 `Ctrl+C` 停止旧雷达进程，再启动 RViz 测试入口：

```bash
cd /home/jetson/luhao/my_nav_carcar
./scripts/run_lidar_humble.sh \
  /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  true
```

RViz 已预配置：

- Fixed Frame：`base_link`；
- LaserScan Topic：`/scan`；
- Reliability Policy：`Best Effort`；
- 视角：TopDownOrtho 顶视图；
- 同时显示 Grid、LaserScan 和 TF。

必须从 Jetson 图形桌面中的终端执行该命令，并先确认 `echo "$DISPLAY"` 有输出。SSH 或无图形自动化终端中 `DISPLAY` 为空时，RViz 会报告 `could not connect to display`；这不代表雷达驱动失败。

在雷达周围分别放置纸箱或墙面，并缓慢改变目标距离。合格现象为：

1. 红色点云轮廓随目标移动而连续变化。
2. 点云方向与雷达周边物体方位一致；若左右镜像，再测试 `inverted:=true`。
3. RViz 的 LaserScan 状态为 `OK`，无 `No transform` 和 `No map received` 报错。
4. 静止场景中点云没有周期性整体跳动或长时间清空。
5. 当前已确认稳定读数范围为 `0.10～3.00 m`；超出该范围不作为本车导航验收能力。

如雷达安装位已经实测，可覆盖测试 TF；以下仅演示参数格式：

```bash
cd /home/jetson/luhao/my_nav_carcar
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_lidar lidar_rviz_test.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  laser_x:=0.0 laser_y:=0.0 laser_z:=0.20 \
  laser_roll:=0.0 laser_pitch:=0.0 laser_yaw:=0.0 \
  use_rviz:=true
```

## 八、停止与恢复

1. 在启动雷达的终端按 `Ctrl+C`。
2. 等待日志显示雷达节点和 RViz 退出。
3. 确认串口没有残留占用：

```bash
cd /home/jetson/luhao/my_nav_carcar
fuser -v /dev/ttyUSB1 2>/dev/null || true
ros2 node list
```

若节点没有退出，先确认进程，再发送普通终止信号：

```bash
pgrep -af 'sllidar_node|lidar_test_rviz'
pkill -TERM -f sllidar_node
```

不要在雷达仍被进程占用时重复启动第二个驱动。

## 九、正式系统接入方式

正式运行时由机器人描述发布 `base_link -> laser_frame`，因此只启动不带临时 TF 和 RViz 的入口：

```bash
cd /home/jetson/luhao/my_nav_carcar
unset PYTHONHOME PYTHONPATH
export PATH="/usr/bin:/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_lidar lidar.launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

在雷达安装尺寸和朝向尚未现场测量前，不把该入口加入 `carcar_bringup robot.launch.py`，避免临时 TF 参数被误认为已标定值。完成 TF 后即可进入正式 bringup，不再增加新的雷达底层实验。

## 十、剩余实测数据

请把以下结果追加到 `docs/测试记录.md` 的 `LIDAR-001`：

- RViz 中车前、车左、车右目标的点云方位是否与实物一致；
- 点云是否左右镜像或前后颠倒；
- 连续运行 10 分钟有无掉线、卡转或异常发热；
- 雷达相对 `base_link` 的实际 `x/y/z/roll/pitch/yaw`。
