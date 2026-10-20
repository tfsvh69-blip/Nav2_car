# carcar_hardware

本包提供Rosmaster控制板的ros2_control硬件插件、协议/运动学逻辑及独立底层测试。插件由一个SystemInterface独占串口；当前正式导航使用carcar_base驱动，不能与该插件同时打开控制板。

## 接口与单位

| 接口 | 含义 |
|---|---|
| left/right_wheel_joint/encoder_count、encoder_velocity | 双轮历史只读模式的M1/M2原始累计计数与count/s，不是弧度或rad/s |
| board_imu/* | IMU状态；底板姿态未验收，采用单位四元数并标记orientation_covariance[0]=-1 |
| motor_1_joint～motor_4_joint | 四端口测试的原始编码器状态与motor_pwm；PWM百分比硬限20%，不包含轮位/麦轮运动学，不能直接接导航 |
| carcar_hardware/RosmasterSystem | 插件XML导出名，URDF与pluginlib动态加载入口保留 |

## 操作与保护

完整构建、启动、观察、停止/断电步骤统一由[操作手册专题索引](../../docs/操作手册.md)进入；低层历史命令及原始说明保留在[本包说明原文](../../docs/archive/carcar_hardware说明原文_20261014.md)，不在README重复维护命令。

- 只读、双轮命令锁定、四电机端口测试入口及encoder_state_monitor均保留；参数和Launch资源完整保留，不因归档删除。
- S1全行程往返实机模式已停用：D435支架可能碰撞；只保留离线预览与历史记录，见手册22节。
- 中位键盘调角与camera_servo_state同步入口保留，见手册23/29节；估算joint state不是实物反馈。
- 测试先关闭其他串口所有者；启停与超时保持零输出，异常优先断执行器动力。串口、轮位、方向和单位按硬件台账确认，不把历史双轮假设套到当前整车。
- 协议、差速换算和不打开真实串口的插件测试由CMake登记；验证与已知边界统一见[测试记录](../../docs/测试记录.md)。

2026-10-14仅合并文档入口；插件、可执行程序、资源安装、保护参数与测试行为均未修改。

2026-10-15 APP-001-R1新增网页逐步调角由正式 `carcar_base` 唯一串口所有者转发，本包键盘／扫描等历史入口保留。网页示教不与独占串口的键盘测试并行；新接口及范围见[巡检应用说明](../carcar_inspection/README.md)，操作统一[手册35节](../../docs/操作手册.md#35-网页巡检与逐步舵机示教app-001-r1)。本轮未执行真实舵机动作。
