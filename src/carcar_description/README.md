# carcar_description

2026-09-24 最新修订：根据 IMG_8557～8562 和三件新 STL 重画舵机、固定脚、可动黑色支架与 D435；黑色支架和两件橙色固定脚的方向已按实物照片复核，前侧固定脚再按用户圈注绕车体 Z 轴翻转 180°。用户已决定建模宽统一为 216 mm。
IMU 位置暂估为相对 `base_footprint=(-40,-15,75) mm`，轴向待验证。当前操作以
[操作手册](../../docs/操作手册.md) 为准。

本包维护四轮麦克纳姆巡检小车的整车 URDF/Xacro、碰撞包络、静态 TF 和 RViz2 查看入口。
模型依据 `实车照片以及相关的模型文件/` 中新旧照片和毫米制 STL 重建，覆盖下层金属双层底盘、
四台电机、四个麦克纳姆轮、Rosmaster 控制板、电池、蓝色顶板/围护件、Jetson 与双天线、
RPLIDAR A1、RealSense D435 系列外壳及安装支架。

正式模型发布入口为 `launch/description.launch.xml`，由唯一的 `robot_state_publisher` 发布
`/robot_description` 与固定 TF；旧 Python 入口按历史资产保留。整车实机启动使用
`carcar_bringup/launch/robot.launch.xml`，完整命令和停止顺序见操作手册第 17.3 节。

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
    └── camera_mount_link（固定舵机壳）
        ├── camera_servo_left_foot_link（后侧橙色固定脚）
        ├── camera_servo_right_foot_link（前侧橙色固定脚）
        └── camera_tilt_link（可动黑色支架）
            └── camera_link（D435 左红外光心）
```

`laser_frame` 是雷达扫描平面中心，名称与 `carcar_lidar` 默认配置一致。本包只发布
`base_link -> camera_mount_link -> camera_tilt_link -> camera_link` 安装链及两件固定脚的固定关节；`camera_depth_frame`、`camera_color_frame` 及各
`optical_frame` 必须由官方 `realsense-ros` 驱动唯一发布。
默认外参文件为 `config/camera_extrinsics.yaml`，采用 2026-09-23 平视粗量初值并标记
`verified: false`。`camera_tilt_dynamic:=false` 默认把轴锁在标定姿态；只有具备实测角度的
`/joint_states` 来源时，才显式设为 `true` 使用受限转动关节。舵机命令值不是角度反馈。
`camera_extrinsics_file` 仍可临时加载候选，候选数值表示零倾角时 `base_link -> camera_link`。

## 尺寸状态

旧固定相机姿态量得整车长 `235 mm`、高 `230 mm`；新俯仰结构扫掠外廓待测。最小离地间隙 `10 mm`；蓝色顶板
`220 × 130 × 10 mm`、用户复核顶面离地 `135 mm`；轮径 `60 mm`、轮宽 `31 mm`、前后轮心距
`120 mm`、左右轮心距 `185 mm`。用户量得整车最大宽 `213 mm`，而轮距加轮宽得到轮组
理论外宽 `216 mm`；复核前 collision/后续 Nav2 footprint 采用较保守的 `216 mm`。

雷达扫描中心相对 `base_footprint` 为 `(-50, 0, 217) mm`，写入相对 `base_link` 的坐标为
`(-0.050, 0, 0.187) m`。RealSense 左红外镜头中心相对 `base_footprint` 为
`(150, 17.5, 155) mm`，写入相对 `base_link` 的坐标为 `(0.150, 0.0175, 0.125) m`；这是平视粗量与厂家镜头偏置推算。
舵机轴心相对 `base_footprint` 为 `(90, 0, 145) mm`，盒体质心为 `(82, -30, 145) mm`；
模型暂以质心作为视觉盒中心；两件固定脚的带孔底面位于离地 135 mm 的顶板上，
分别靠舵机盒前后端，前件已按用户圈注绕车体 Z 轴翻转。用户复核银色 D435 外壳左右中心线
与舵机输出轴横向对齐；官方机械图中左红外光心位于外壳中心线的车体左侧约 17.5 mm，
先前 `Y=+40 mm` 指的是用户正面观看时最左边的圆孔，不能作为左红外光心。20KG 级舵机的壳体包络为照片估算，
实际型号、支架安装面位置和俯仰扫掠外廓待测；旧整车 235 mm 长度不覆盖新相机前伸。
RealSense 外壳 `90 × 25 × 25 mm`、RPLIDAR A1 Dev Kit 最大外廓
`96.8 × 70.3 × 55 mm` 和 NVIDIA P3768 载板 `100 × 79 mm` 来自厂家机械资料。
正面四窗口由两个红外成像头、一个红外投射器和一个 RGB 彩色镜头组成；
从正面看，左起依次为右红外、投射器、左红外、RGB 彩色镜头。镜头分工及左红外光心基准参考
[RealSense D400 系列数据表（2025-10）](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf)。

IMU 轴向、相机精确 pitch/yaw、Jetson 外壳细节和质量/惯量仍待确认。
旧相机安装曾以 `roll=pi` 记为倒装；最新正面照片的前窗排列和低负载彩色直播帧方向均支持
新安装采用 `roll=0` 候选，仍需连同 TF 在现场核对。黑色支架属于
`camera_tilt_link`，与相机一起绕舵机 Y 轴转动。雷达平面方向已由 RViz 与实物对照通过。
集中在 `urdf/carcar_dimensions.xacro`。未确认值不能作为实机标定或动力学验收结论。
测量和 IMU 轴向动作判定方法见
[`docs/操作手册.md`](../../docs/操作手册.md)。

## 构建与查看

完整构建、启动、观察和停止命令统一维护在[操作手册第 24 节](../../docs/操作手册.md#24-新舵机相机结构模型与-tf-离线检查)。
Windows 系统临时查看车模时，可在 Jetson 桌面通过 `description.launch.xml` 的
`use_rviz:=true` 显式打开模型配置；已有 `robot_state_publisher` 时同时设置
`publish_model:=false`，避免重复发布。完整步骤见[操作手册第 24.2 节](../../docs/操作手册.md#242-windows-系统下临时在-jetson-桌面看车模)。
纯车模预览还可设 `preview_wheels:=true`，临时发布四轮零角度以补齐轮关节 TF；
车体或其他节点发布真实 `/joint_states` 时不得使用这个预览值。雷达蓝色底板按
顶板顶面离地 135 mm 放置；用户实测雷达黑色下底板底面高于顶板 39 mm，扣除蓝色底板
5 mm 厚度后，模型蓝色支柱长度为 34 mm。
黑色相机活动支架按实物方向安装：STL 的小端轴孔对准舵机输出轴，大平面朝车头连接 D435 背面；
此方向只经过离线几何与 RViz 核对，紧固件和实际接触间隙仍待实物复核。
两件橙色舵机固定脚分别用固定关节连接到 `camera_mount_link`：一件靠在舵机盒后端，
另一件靠在前端；STL 带双孔的平底贴蓝色顶板，两个螺孔沿车体 Y 轴排列。
前端固定脚的视觉外缘约超出当前顶板前缘 2 mm，螺孔中心仍在板内；此处仍是照片约束的近似模型。
2026-09-24 用户圈出靠相机一侧固定脚，要求绕车体 Z 轴翻转 180°；模型保留了该件的平面中心和贴板高度。

## 外部资产与依据

轮子视觉 STL 来自 Apache-2.0 许可的 MOGI-ROS，并按实物缩放，固定提交与许可证见
`third_party/mogi_ros/NOTICE.md`。RPLIDAR、RealSense 和 Jetson 本体使用本项目自建的轻量
几何，不复制厂家 CAD；机械资料链接及采用尺寸记录在整车测量文档中。用户自制
`.SLDPRT` 导出的毫米制 STL 已复制到 `meshes/vehicle/`，原名、外廓、SHA-256 和安装变换
见该目录的 `README.md`；本轮未改写用户源文件，当前参考目录为 STL，SLDPRT 请在设计端保留。
