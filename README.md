# carcar 四轮麦克纳姆导航巡检小车

最后更新：2026-09-14

面向 ROS 2 Humble (Ubuntu 22.04 LTS) 的四轮麦克纳姆室内巡检小车。业务目标涵盖室内自主建图、静态地图定位、Nav2 单点与多点自主导航巡检、Groot 实时行为树监控，以及 RealSense D435 视觉辅助回充。

当前进度、最新结论与唯一下一步请以 [项目现状与路线](docs/项目现状与路线.md) 为准；所有实机构建、启动、观察与应急停止命令统一收敛于 [操作手册](docs/操作手册.md)。

---

## 核心技术选型与当前状态

* **计算平台**：NVIDIA Jetson Orin NX，Ubuntu 22.04 LTS，ROS 2 Humble。
* **运动模型**：四轮麦克纳姆轮底盘，因带载重载静摩擦大，软件层正式锁定为**非全向差速模型**（`holonomic=false`，禁用横移），保障行驶轨迹与里程计的高精度与高重复性。
* **底盘控制**：STM32 控制板板载闭环运动 PID（`Kp=8.0, Ki=1.2, Kd=0.8` 已固化入 Flash），四轮编码器标定完成，底层硬件看门狗 `0.30 s`。
* **里程计基线**：四轮累计编码器纯轮式里程计（1 m 直行与 360° 原地自转实测通过，静止零漂移）。严格遵循状态估计推进规范，IMU 虽已完成 REP-103 驱动校正，但当前**保持纯轮式里程计独立输入，暂不接入滤波融合**。
* **建图与导航**：
  * SLAM Toolbox 异步建图通过（NAV-001/002/004），已建立覆盖大范围的高清栅格地图（`nav004_20260914_093032`）并通过独立 C++ 重载比对校验。
  * Nav2 差速导航栈就绪（AMCL 差速模型、DWB 控制器、实车贴合包络 `0.28x0.26m`）。
  * 全局膨胀层开启（`0.30 m / 5.0`），确保规避垃圾桶等突发障碍时保有大于半车宽的安全净空；局部膨胀维持 `0.15 m / 5.0`。
  * 状态监控与会话归档：Groot 1 与桥接监视器已正式停用并默认关闭；全面改由 C++ `nav_event_logger` 提供 1 Hz 终端中文摘要、阶段流转即时告警，并在 `log/nav_sessions/` 自动生成含 1 GiB 分卷 rosbag2 的完整会话包（带 10 GiB 与磁盘 <2 GiB 保护）；配套 C++ 工具 `nav_log_tool` 提供会话清单、时序时间线与纯离线零发布回看。
  * 当前正在开展 NAV-010（垃圾桶动态绕障）与 NAV-011（终端状态与会话录包系统）实车现场验收。

---

## 软件架构与分层

```text
硬件串口 (/dev/myserial, /dev/ttyUSB0)
        ↓
rosmaster_vendor       原厂底层通信封装（第三方驱动）
        ↓
carcar_base            电机动力学、纯轮式里程计、底层安全看门狗
        ↓
carcar_description     URDF/Xacro 模型、实车物理尺寸与静态 TF 树
        ↓
carcar_lidar           RPLIDAR A1 雷达驱动编排与点云滤波
        ↓
carcar_navigation      SLAM 建图、AMCL 定位、Nav2 导航参数、nav_event_logger (状态与会话录包)、nav_log_tool 分析工具、XML Launch
        ↓
carcar_bt_monitor      【已停用】早期基于 BehaviorTree.CPP ZMQ 的 Groot 1 桥接（保留作历史测试对照）
        ↓
carcar_algorithms      感知与特征提取通用算法（规划中）
        ↓
carcar_inspection      巡检业务状态机、多点调度与视觉回充逻辑（规划中）
        ↓
carcar_bringup         整车启动编排
```

---

## 文档架构

项目严格遵守文档收敛约定，`docs/` 根目录仅维护当前有效的 5 份核心文档：

| 核心文档 | 用途说明 |
|---|---|
| [文档索引](docs/文档索引.md) | 文档总入口与维护规范 |
| [项目现状与路线](docs/项目现状与路线.md) | 当前阶段状态、实车/离线结论表、唯一下一步与推进顺序 |
| [硬件台账](docs/硬件台账.md) | 已确认的硬件连接、端口、实测尺寸、标定参数与待确认事实 |
| [操作手册](docs/操作手册.md) | **全项目唯一命令维护入口**：环境加载、编译、节点启动、观察与安全停机 |
| [测试记录](docs/测试记录.md) | 自底向上各测试项（DRV/MOTOR/IMU/ODOM/TF/NAV）详细过程与结果归档 |
| [历史资料](docs/历史/README.md) | 早期专题调研、旧方案、停用命令与系统架构审计文档归档 |

*注：系统级 Ubuntu/Jetson 配置变更记录请查阅桌面维护文档 `/home/jetson/Desktop/系统配置维护记录.md`。*

---

## 快速构建

离线编译整车核心功能包（不访问串口、不触动电机）：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

# 构建底盘、描述、雷达与导航核心包
colcon build --symlink-install --packages-select \
  rosmaster_vendor carcar_base carcar_description carcar_lidar carcar_navigation carcar_bt_monitor

source install/setup.bash
```

完整的多终端启动、传感器联调、SLAM 建图、Nav2 导航与终端状态录包操作流程，请直接查阅 **[操作手册](docs/操作手册.md)**。
