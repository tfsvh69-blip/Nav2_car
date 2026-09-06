# carcar_hardware

本包提供 Yahboom Rosmaster 控制板的 `ros2_control` Hardware Component，由一个
`hardware_interface::SystemInterface` 独占 `/dev/myserial`。已保留只读验证入口，
另有默认关闭硬件运动锁的双轮历史测试入口和四电机端口级测试入口。

## 只读接口

- `left_wheel_joint/encoder_count`：M1 左轮累计原始计数。
- `left_wheel_joint/encoder_velocity`：M1 左轮原始计数变化率，单位暂为 count/s。
- `right_wheel_joint/encoder_count`：M2 右轮累计原始计数。
- `right_wheel_joint/encoder_velocity`：M2 右轮原始计数变化率，单位暂为 count/s。
- `board_imu/*`：标准 IMU 姿态、角速度和线加速度状态接口。底板当前没有经过验证的姿态，因此使用单位四元数并设置 `orientation_covariance[0]=-1`。

只读入口中的 `encoder_count` 不能当作弧度，`encoder_velocity` 不能当作 rad/s。

四电机测试模式按控制板端口导出 `motor_1_joint`～`motor_4_joint`，每路包含原始
`encoder_count`、`encoder_velocity` 状态和 `motor_pwm` 命令接口。`motor_pwm` 单位为
PWM 百分比，硬限制不超过 20%；它不是标准轮速接口，不包含轮位、方向或麦克纳姆
运动学，不能直接供导航使用。

## 命令测试接口

命令测试入口另外导出左右轮标准 `position`、`velocity` 状态和 `velocity` 命令接口，
并可加载 `diff_drive_controller`。当前使用的 `1320 count/rev`、`0.05 m` 轮半径和
`0.20 m` 轮距都是临时测试值，只用于确认数据链路。

`motion_enabled` 默认是 `false`，此时即使命令接口被控制器认领，插件也只发送零速。
显式启用后，等轮速的正向短脉冲已经实机通过。M1/M2 目前接在原厂固件定义的同一侧
端口组，差速转向尚不成立，因此当前命令入口只算架空直线功能测试，不可用于导航。

## 构建

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --symlink-install \
  --packages-select carcar_hardware carcar_base \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 安全启动

启动前不得运行 `carcar_base/rosmaster_node.py` 或其他打开底板串口的程序。插件配置、
激活、停用和退出时会发送零速；以下只读入口不导出命令接口。

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
fuser -v /dev/ttyUSB0 /dev/myserial 2>/dev/null || true
ros2 node list
ros2 launch carcar_hardware read_only_hardware_test.launch.py
```

## 观察

另开终端执行：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /dynamic_joint_states --once
ros2 topic echo /imu_sensor_broadcaster/imu --once
```

`command interfaces` 下必须为空。详细频率检查、停止和结果记录见 `docs/测试记录.md` 的 `DRV-002`。

## 默认锁定的命令入口

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_hardware diff_drive_hardware_test.launch.py
```

默认硬件运动锁关闭。只有在车轮可靠架空时，才允许按 `docs/测试记录.md` 的
`DRV-003` 显式使用 `motion_enabled:=true` 做固定直线短脉冲；当前不要发布转向命令。

手转编码器时可使用只读观察器。它只订阅
`/dynamic_joint_states`，不会打开串口或发送电机命令：

四路反馈使用以下默认锁定入口：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carcar_hardware four_motor_hardware_test.launch.py
```

随后另开终端运行观察器，可同时查看 M1～M4；完整步骤见 `docs/测试记录.md` 的
`ENCODER-003`。

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run carcar_hardware encoder_state_monitor.py
```

## 停止与恢复

在启动终端按 `Ctrl+C`，再确认节点和串口均已释放：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
fuser -v /dev/ttyUSB0 /dev/myserial 2>/dev/null || true
```

如果车轮意外动作或软件停车失效，立即断开电机主电源。当前尚未验证进程被
`SIGKILL` 或 Jetson 突然掉电时的底板固件级停车，不得无人值守运行。
