# carcar_description

2026-09-07 最新修订：用户已决定建模宽统一为 216 mm，原 213 mm 差异不再作为待办。
IMU 位置暂估为相对 `base_footprint=(-40,-15,75) mm`，轴向待验证。当前操作以
[操作手册](../../docs/操作手册.md) 为准。

本包维护四轮麦克纳姆巡检小车的整车 URDF/Xacro、碰撞包络、静态 TF 和 RViz2 查看入口。
模型依据 `实车照片以及相关的模型文件/` 中 4 张实车照片和 5 个毫米制 STL 重建，覆盖下层金属双层底盘、
四台电机、四个麦克纳姆轮、Rosmaster 控制板、电池、蓝色顶板/围护件、Jetson 与双天线、
RPLIDAR A1、RealSense D435 系列外壳及安装支架。

## 坐标与 TF 所有权

坐标遵循 REP-103：X 指向车头、Y 指向车体左侧、Z 向上。`base_footprint` 位于四轮接地点
区域的地面中心；`base_link` 位于四个轮心构成矩形的中心并与轮轴等高。

```text
base_footprint
└── base_link
    ├── front_left_wheel_link
    ├── front_right_wheel_link
    ├── rear_left_wheel_link
    ├── rear_right_wheel_link
    ├── upper_body_link
    ├── compute_link
    ├── imu_link
    ├── laser_frame
    ├── camera_link
    └── camera_mount_link
```

`laser_frame` 是雷达扫描平面中心，名称与 `carcar_lidar` 默认配置一致。本包只发布
`base_link -> camera_link` 安装外参；`camera_depth_frame`、`camera_color_frame` 及各
`optical_frame` 必须由官方 `realsense-ros` 驱动唯一发布。

## 尺寸状态

已实测确认：整车最大长 `235 mm`、最大高 `230 mm`、最小离地间隙 `10 mm`；蓝色顶板
`220 × 130 × 10 mm`、顶面离地 `140 mm`；轮径 `60 mm`、轮宽 `31 mm`、前后轮心距
`120 mm`、左右轮心距 `185 mm`。用户量得整车最大宽 `213 mm`，而轮距加轮宽得到轮组
理论外宽 `216 mm`；复核前 collision/后续 Nav2 footprint 采用较保守的 `216 mm`。

雷达扫描中心相对 `base_footprint` 为 `(-50, 0, 217) mm`，写入相对 `base_link` 的坐标为
`(-0.050, 0, 0.187) m`。RealSense 左红外镜头中心相对 `base_footprint` 为
`(130, 33, 170) mm`，写入相对 `base_link` 的坐标为 `(0.130, 0.033, 0.140) m`。
RealSense 外壳 `90 × 25 × 25 mm`、RPLIDAR A1 Dev Kit 最大外廓
`96.8 × 70.3 × 55 mm` 和 NVIDIA P3768 载板 `100 × 79 mm` 来自厂家机械资料。

IMU 轴向、相机精确 pitch/yaw、Jetson 外壳细节和质量/惯量仍待确认。
用户在实时画面中确认相机物理倒装，`camera_link` 已按绕车体前向 X 轴
`roll=pi` 修正；支架视觉模型单独放在不旋转的 `camera_mount_link`，避免把实物支架
也倒转。雷达平面方向已由 RViz 与实物对照通过。
集中在 `urdf/carcar_dimensions.xacro`。未确认值不能作为实机标定或动力学验收结论。
测量和 IMU 轴向动作判定方法见
[`docs/操作手册.md`](../../docs/操作手册.md)。

## 完整构建与离线检查

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m py_compile \
  src/carcar_description/launch/description.launch.py \
  src/carcar_description/launch/view_model.launch.py \
  src/carcar_description/test/test_description.py
xacro src/carcar_description/urdf/carcar.urdf.xacro > /tmp/carcar_model.urdf
check_urdf /tmp/carcar_model.urdf
/usr/bin/colcon build --symlink-install --packages-select carcar_description
/usr/bin/colcon test --packages-select carcar_description --event-handlers console_direct+
/usr/bin/colcon test-result --verbose
source install/setup.bash
```

## 启动、观察与停止

模型入口不打开串口，不启动电机、雷达或相机驱动。启动前仍应关闭其他同名
`robot_state_publisher`，避免误判重复 TF。

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 launch carcar_description view_model.launch.py
```

另开终端观察：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link camera_link
```

每条 `tf2_echo` 得到稳定变换后按 `Ctrl+C`，再执行下一条。停止时先关闭 RViz2，再在 launch
终端按 `Ctrl+C`，最后确认：

```bash
cd /home/jetson/luhao/my_nav_carcar
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
pgrep -af '[c]arcar_model_rviz|[m]odel_joint_state_publisher|[r]obot_state_publisher' || true
```

## 外部资产与依据

轮子视觉 STL 来自 Apache-2.0 许可的 MOGI-ROS，并按实物缩放，固定提交与许可证见
`third_party/mogi_ros/NOTICE.md`。RPLIDAR、RealSense 和 Jetson 本体使用本项目自建的轻量
几何，不复制厂家 CAD；机械资料链接及采用尺寸记录在整车测量文档中。用户从 5 个自制
`.SLDPRT` 导出的毫米制 STL 已复制到 `meshes/vehicle/`，原名、外廓、SHA-256 和安装变换
见该目录的 `README.md`；本轮未改写用户源文件，当前参考目录为 STL，SLDPRT 请在设计端保留。
