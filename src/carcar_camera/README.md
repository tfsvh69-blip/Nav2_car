# carcar_camera

94档静止STVL验收使用本次地面一致性外参 `camera_extrinsics_level94_ground_candidate_20261008.yaml`（verified=false），保留原生宽视场点云。RViz当前验收配置和零参考100刷新候选均保存Frame Rate=100；完整判据/操作见[手册31.16](../../docs/操作手册.md#3116-94档近平视与100刷新率的静止stvl验收)。

断电/重启或姿态变动后，平视角检查使用零旋转参考TF与 `rviz/camera_floor_zero_pitch_reference_100fps_candidate.rviz`；4话题只读桥见 `config/floor_reference_display_bridge_candidate.yaml`，原10刷新配置保留作历史。先检查原生测量地面斜率，不能用已补偿TF的平地画面推断相机平视。测角操作及适用前提见[手册31.15](../../docs/操作手册.md#3115-断电或硬件移动后重新核对平视角)，100刷新覆盖及当前静止地图入口见31.16，原候选和测试资产保留。

原生深度与RGB属于不同镜头，视场差异是正常现象。导航保留原生深度生成的宽视场点云；RGB配准深度只作显示/认物参照，不替换导航输入或以调安装TF消除视场差异。三窗用途、当前640×480内参与官方HD规格的区别、黑色无效深度判读统一见[手册31.11](../../docs/操作手册.md#3111-深度配准到-rgb-的显示参照nav-016-r2)，等大布局见31.12。

`d435_navigation_high_accuracy_rgb_candidate.yaml`复用官方High Accuracy=3测距预设，配合现有标障时间确认，默认导航不改；独立Spatial候选用于对照，不叠加启用。C++手动入口`depth_device_settings`按精确序列号保存/恢复SDK高级JSON，不开启流、切换高级模式或复位USB；原Custom必须完整备份，不能仅设回0恢复。`depth_snapshot`新增可选`central_half_width`（默认0.15 m不变），可扩展边缘只读CSV取证，原地面统计范围不变。构建、候选启停、观察、备份与回滚完整命令仅见[手册31.10](../../docs/操作手册.md#3110-深度噪点定位与高置信度测距候选nav-016-r2)，证据NAV-016-R2；本轮转向使滤波效果对照不满足单变量，不宣称假块已修复。

`image_reference_preview`为显示补充深度灰度PNG/RGB JPEG各320×240/1 Hz，保留各自header；复用现有OpenCV与标准CompressedImage协议，原始测量不改。深度灰度不可作毫米测距、两图未配准。XML`image_reference_preview.launch.xml`只读，不开启相机；`camera_floor_check`仅增加可选driver_params_file，默认深度单流不变。构建、RGB开启/关闭、桌面插件、域桥及RViz完整命令统一见[手册31.9](../../docs/操作手册.md#319-rgb-与深度压缩参照窗口nav-016-r2)，结果NAV-016-R2。

`depth_temporal_audit`为未压缩静止包提供只读时间确认影子对照，复用既有2 cm格/3帧/2次命中逻辑；保留原标障/清障和旧测试资产。`depth_filter_stvl_temporal_candidate.yaml`相对STVL输入基线仅启用确认，高度、TF、TTL与健康门槛不变；仍为独立实验候选，移动/小目标不可从离线盒子保留推定通过。构建、启停、回滚和验收命令只维护于[手册31.7](../../docs/操作手册.md#317-当前98档普通盒静止验收与显示负载隔离nav-016-r2)，结果NAV-016-R2。

2026-10-07 静止STVL负载对照：异步发布候选出现44.47秒接收间隔，`camera_floor_async_candidate.launch.xml`已停用并保留证据，继续使用同步入口。`depth_cloud_preview`同时可用于STVL低带宽显示；必须转发结构有效的空点云，否则预览可能残留旧帧。它不供导航/验收计数使用，畸形缓冲仍拒绝；新增真实进程回归测试验证非空→空更新。操作入口见[手册31.7](../../docs/操作手册.md#317-当前98档普通盒静止验收与显示负载隔离nav-016-r2)，结果见NAV-016-R2。

新增只读C++ `depth_stvl_audit`读取已封存的未压缩bag中原生STVL PointCloud2，按固定base_footprint三维ROI输出逐帧完整体素计数、消息时间戳和录包接收时间。拒绝旧VoxelGrid类型、错误坐标系、畸形缓冲及非有限坐标，接受有效空帧；不修改TF、参数、地图或硬件，不直接判断健康/物理空闲。独立`test_stvl_roi`覆盖XYZ各轴筛选、空更新、畸形/非有限点及错误帧拒绝；原VoxelLayer和二维审计入口保留，完整命令31.7。

三维审计另存同前缀`_roi_voxels.csv`，逐体素保留消息时间与原生XYZ，便于核对低点位置/高度；原计数CSV列与CLI保持。两份输出均拒绝覆盖、任一写入失败整次返回失败，原始输入不变。ROI遍历与计数共用字段/缓冲/有限值验证，单测追加坐标一致性验证。

2026-10-07 接手续测：新增 `rviz/depth_image_load_candidate.rviz`，仅订阅原生深度图，用于在无头基线完成后分离跨机显示负载；不是日常导航配置，不改变相机或过滤默认。当前现场为台式机 `luhao@Hao/192.168.5.17`，笔记本bao不在现场。完整操作见[手册19.13](../../docs/操作手册.md#1913-当前台式机深度图负载单变量对照nav-016-r2)，健康仍按30.17。

独立 `rviz/camera_floor_current_20261007.rviz` 关闭旧体素显示，只显示1 Hz/最多8000点的 `/camera/debug/points` 及模型/TF；完整启动、QoS及停止见[手册19.12](../../docs/操作手册.md#1912-当前ubuntu主机查看低带宽地面侧视)。用户授权逐档S1对照见[29.5](../../docs/操作手册.md#295-当前姿态逐档测量对照2026-10-07)，相同档位角度重复性失败后已停键盘。调角前候选、回退后独立候选和零旋转计算参考均不是正式外参；命令估计不能代替位置反馈，实测只登记NAV-016-R2。

2026-10-07：新增可选 `d435_navigation_rgb_reference_candidate.yaml`，深度保持原640×480/6 Hz，彩色参照为640×480/6 Hz；导航默认仍为深度单流。RGB显示不是健康判据，使用及回退见[操作手册31.6](../../docs/操作手册.md#316-rgb-参照安装角与更快衰减对照nav-016-r2)。相机姿态改变后须重新测固定外参，不能继续套用旧候选。

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

`depth_snapshot` 支持 `sample_frames`（默认3，1～60）与 `sample_interval_ms`（默认0，0～5000），保存每帧时间戳到 `frames.csv`，其余旧默认保持。新增 C++ 离线工具 `depth_floor_profile` 读取多个图像投影CSV，输出0.5～1、1～1.5、1.5～2、2～3 m的逐帧及合并高度统计、实际覆盖与近距二维平面残差；不按高度删除异常点，不自动修改外参或导航阈值。空段记 `nan`，少量点不能代表整段可用。复用已安装OpenCV 4.10.0的`solve/DECOMP_SVD`，不新增生产过滤算法。完整命令与舵机姿态策略见[操作手册30.16节](../../docs/操作手册.md#3016-保持当前舵机姿态分距离地面验收与转正门槛)。

`camera_floor_check.launch.xml` 新增可选`filter_params_file`，默认仍为旧`depth_filter.yaml`；`depth_filter_near_2cm_candidate.yaml`只用于未验收的近距2 cm低障碍对照，与导航包`depth_costmap_near_2cm_candidate.yaml`配套，两级下限1 cm且`mark_threshold=0`。标障光心斜距暂限0.85 m，清障仍2 m，不可作为正式导航或全视野保证。

2026-10-05 新增 `camera_floor_check.launch.xml`：固定当前姿态，使用深度单流，不驱动舵机或车轮。`depth_cloud_preview` 仅把原始点云均匀抽样为最多8000点、最高1 Hz的 `/camera/debug/points`，供主机侧视；不接入标障/清障。对应 `camera_floor_check.rviz` 使用 `base_footprint`、Z着色、Best Effort/Volatile/队列1、默认侧视，并保留俯视/前视。全量测量仍由 `depth_snapshot` 读取原始数据；新增可选 `central_max_x`、`floor_min_row`、`floor_max_row`，默认1.2 m/360/450保持原采样范围，中央2 m空地检查可显式使用2.0/240/479。地面姿态与数据新鲜度分别验收；地面可见或低于3 cm不能证明平视。流程见[操作手册第30.15节](../../docs/操作手册.md#3015-落地后的地面测量与独立外参候选tf-006-r3)。

`camera_tf_verify.launch.xml` 只启动固定平视模型、RGB/深度驱动和已有点云诊断，不启动底盘、舵机或深度层监督器。专用驱动配置保持 640×480、6 FPS，点云不对齐到 RGB。默认 `use_rviz=false`，Hao 使用安装到包内的 `rviz/camera_tf_verify.rviz`，Fixed Frame 为 `base_footprint`。默认模拟四轮零角度只供静止模型，复用真实模型时设置 `publish_model=false`。候选通过 `camera_extrinsics_file` 加载，仍保留 `verified: false`；只看 RGB 对准不能证明安装外参正确。完整四终端命令和停止、回退只维护在[操作手册第 27 节](../../docs/操作手册.md#27-d435-不用标定板的静止-tf-粗校验tf-006)。

`depth_health_window`是只读C++进程，连续健康3秒后锁定600秒窗口，统计每分钟频率、点云接收间隔/时间戳年龄和过滤器诊断；异常不刷新计时，中断不能通过。保留单点时间信息而不录大点云，独立目录防覆盖。合成消息集成测试覆盖通过、原始流中断、诊断超时和提前取消。

`depth_target_window`是手动启动的小目标只读取证入口：在当前TF下对同一车体ROI分别统计原始和过滤点云，保存逐帧点数、时间戳年龄、接收间隔与自身计数耗时。最多15秒等待两流新鲜且TF有效，随后固定60秒，缺流/过期/中断不能形成有效观测；不保存全量点云、不发运动指令、不被Launch隐式启动。`frames_ge1/ge3`是比较指标，不代表STVL或目标验收通过，必须配对同位置空场、现场目标位置和独立健康窗口。完整构建/操作/停止见[手册29.6](../../docs/操作手册.md#296-小方块原始与过滤点数对照)。

`depth_floor_profile`保留旧统计列并追加单点`p999_z_m`，同时输出近距和0.5～3 m整段平面；近距点密集可能主导整段拟合，不等于远段验收。新配置`d435_navigation_decimation_candidate.yaml`仅试验官方SDK抽样2；`fastdds_camera_shm_candidate.xml`仅由本轮进程显式加载，为Fast DDS 2.6.12设置16 MiB共享内存并保留UDP，不改系统或话题QoS默认。共享内存大于消息的依据见[官方说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)。

新增独立诊断双源`depth_filter_near_far_candidate.yaml`配对导航包`depth_costmap_near_far_candidate.yaml`：同一个标障话题，近路0.85 m/1.5 cm、远路2 m/5 cm，斜距与高度分别筛选；2026-10-05已用于空场和2 cm正方体静止首轮，完整清障尚未通过，不进入正式默认。不保证1.2 m或整个2 m内的2 cm障碍。完整构建、试验及恢复命令只维护在[手册30.17节](../../docs/操作手册.md#3017-相机独占点云超时定位与十分钟健康窗口)。

## STVL 主线实验

新增 `depth_filter_stvl_experimental.yaml`，监督器 `mode=required_stvl` 联合相机健康、推进的点云采样时间与本地/全局STVL消费诊断。异常关闭运动，稳定3秒后恢复有效任务；原 `legacy_switch` 模式保留。近源最低1.5厘米/0.85米、普通源5厘米/2米都是待实机验证值。完整步骤见[操作手册第31节](../../docs/操作手册.md#31-stvl-深度导航主线nav-016)。

显示配准新增`reference_camera_info`和`image_reference_alignment.launch.xml`：复用depth_image_proc 3.0.9，目标网格与320×240 RGB预览一致，保留原生宽视场导航输入和无效深度。真实配准进程离线回归通过，实机输出尺寸已确认；新网格屏幕精度与健康待验。完整命令只见[手册31.11](../../docs/操作手册.md#3111-深度配准到-rgb-的显示参照nav-016-r2)。
