# carcar_camera

本包提供 D435 的实验性导航接入，不由正式整车入口隐式启动。

## 数据接口

- `/camera/depth/points`：RealSense 原始无纹理深度点云的稳定项目话题。
- `/camera/navigation/obstacles`：按 `base_footprint` 高度与车体包络筛选后的标障点云。
- `/camera/navigation/obstacles_ror_shadow`：手动启动 `depth_ror_shadow` 时才发布的 PCL 半径离群点候选，保持输入时间戳与光学坐标系。独立 B Costmap 仍订阅原标障点云，此影子输出只供比较。
- `/camera/navigation/ror_shadow/diagnostics`：每帧输入、保留、删除点数与处理耗时；`SHADOW_ONLY` 不表示正式导航健康验收通过。
- `/camera/navigation/clearing`：保留有效地面和背景测量的清障点云。
- `/camera/navigation/healthy`：原始点云时间戳、格式和 TF 均有效时为 `true`；空场景不算掉线。
- `/camera/navigation/diagnostics`、`/camera/navigation/fusion_status`：筛选与 `LASER_ONLY / RECOVERING / FUSED` 状态。
- `/camera/navigation/status_marker`：供轻量 RViz 显示融合、降级、恢复或切换失败状态的文本标记。

导航入口使用 `camera_navigation.launch.xml`，标定入口使用
`camera_calibration.launch.xml`；两者不得同时运行。驱动固定参考
`realsense-ros 4.58.3`（Apache-2.0）和本机 librealsense 2.58.4，源码导入清单在
`dependencies/realsense-ros.repos`。
注意：当前环境下的点云滤镜名称为 `Pointcloud (Neon)`，ROS 参数前缀必须为
`pointcloud__neon_.*`（已写入 `d435_navigation.yaml`）。
导航深度层监督器默认不启动；完成稳定性与固定姿态 TF 验收后，才在实验入口显式设置
`start_supervisor:=true`。当前粗尺量阶段保持关闭。
阶段一（点云驱动与滤波数据通路）已于 2026-09-24 实机打通；阶段二（Costmap 深度层与动态导航）暂未进行。

## 外参标定

标定板参数固定为 7×5 格、方格 0.030 m、标记 0.022 m、`DICT_4X4_50`。
默认 `measurement_mode=three_points`。`board_reference_points` 依次填写板外框左上角、
右上角、左下角在 `base_footprint` 中的 XYZ，共 9 个米制数；工具会检查两条边是否接近
210 mm、150 mm 且近似垂直，并由三个非共线点求出 `base_footprint → board`。只有外部已经
用同一组基准严谨求出六自由度位姿时，才使用 `xyz_rpy` 模式。设置当前板位参数并调用
`/camera/calibration/capture_pose`；至少采集三个
`calibration` 板位和一个 `validation` 板位后调用 `/camera/calibration/solve`。结果只写入
候选 YAML，包含每个板位的测量记录且默认 `verified: false`，不能自动覆盖正式外参。
当前候选还记录 `mount_condition.servo_target_angle_deg=140.5`、
`servo_position_feedback=false` 和 `measurement_quality=ruler_coarse`。目标角只是发给板端的
进行目标；舵机没有位置反馈，标定前须在现场确认相机停稳，同一候选只适用于该固定姿态。
普通尺量不构成 10 mm、0.5° 绝对精度验收。

完整构建、操作、停止与恢复命令只维护在 `docs/操作手册.md`。

`depth_snapshot` 是手动启动的只读诊断入口，不被 Launch 自动启动，不发布话题或修改参数。采集 3 帧原生深度、彩色参照及基于实时 TF 的高度图，保留毫米原始深度 PNG、中央区域坐标 CSV。依赖现有 CameraInfo 与 TF；15 s 超时退出，输出仅用于定位缺测/高度过滤，不认证外参精度。图像保持原生深度视角，不与 RGB 像素对齐；命令维护在操作手册第 28.6 节。

`depth_map_audit` 是独立的 C++ 离线审计入口，只读已有 bag 的完整 `/costmap/costmap`，逐帧统计指定目标区域和外扩周边的空闲、未知、非零及致命二维栅格，并输出同一位置逐帧的非零代价格子。不依赖原始点云，也不替换旧 `depth_costmap_audit` 的体素与射线诊断；可用于检查二维清障及两侧家具占用是否保持。读取压缩 bag 时会生成临时未压缩副本，磁盘检查及完整命令见[操作手册第 30.7 节](../../docs/操作手册.md#307-完整二维地图补审与轻量静止录包)。

点云筛选新增 `max_clearing_range`，默认跟随 `max_range`，保持既有行为。独立清障候选允许有效背景深度到 4 m、障碍仍限 2 m；无效/超范围深度不能生成清障点，不制造空闲。候选只经 `filter_params_file` 显式加载，不改默认配置；Nav2 清障下限候选 -3 cm 与地面负高度残差相关，标障高度仍 3～26 cm。操作与回退见手册第 28.8 节。

## 不用标定板的静止 TF 粗校验

`camera_tf_verify.launch.xml` 只启动固定平视模型、RGB/深度驱动和已有点云诊断，不启动底盘、舵机或深度层监督器。专用驱动配置保持 640×480、6 FPS，点云不对齐到 RGB。默认 `use_rviz=false`，Hao 使用安装到包内的 `rviz/camera_tf_verify.rviz`，Fixed Frame 为 `base_footprint`。默认模拟四轮零角度只供静止模型，复用真实模型时设置 `publish_model=false`。候选通过 `camera_extrinsics_file` 加载，仍保留 `verified: false`；只看 RGB 对准不能证明安装外参正确。完整四终端命令和停止、回退只维护在[操作手册第 27 节](../../docs/操作手册.md#27-d435-不用标定板的静止-tf-粗校验tf-006)。
