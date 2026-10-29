# Carcar 导航

**2026-10-27 NAV-018：**纯激光实验参数在 `config/nav2_lidar_experimental.yaml`，由巡检入口 `inspection_lidar.launch.xml` 加载。定位与代价图订阅 `/scan/filtered`，控制器前进上限 0.10 m/s、角速度 0.36 rad/s，数值仍待实机。本节其余 STVL 与深度入口保留为历史实验。构建、启动、停止和回滚见[手册39节](../../docs/操作手册.md#nav-018-operation)。

**2026-10-29 NAV-018-R18 录包修复：**`nav_event_logger` 对 `/tf_static` 显式使用 TF2 `StaticListenerQoS`，让先于静态发布者被发现的记录器也能请求保留消息；其他话题继续使用原 rosbag2 QoS 适配。回归启动真实日志进程、仅发布一次合成静态变换，在正常关闭后的录包核对消息与坐标值，覆盖发布者先后两种启动顺序。仅增加已有 Humble `tf2_msgs` 的显式测试依赖，无新运行框架。修复不补写历史包，也不代表返程障碍已消除；结果与完整操作见[测试记录R18](../../docs/测试记录.md#nav-018-r18)、[手册39.13](../../docs/操作手册.md#3913-历史扫描累积回放与静态tf录包回归nav-018-r18)。

当前94档静止验收使用 `rviz/depth_costmap_stvl_level94_acceptance_100fps_candidate.rviz`，保存Frame Rate=100；相机/预览/地图频率分别维护。当前姿态新TF、空地/保持/撤障/健康判据和完整操作只见[手册31.16](../../docs/操作手册.md#3116-94档近平视与100刷新率的静止stvl验收)，历史配置与失败保留。

当前100刷新验收配置已包含原生深度点云预览，配合9话题 `config/stvl_display_depth_cloud_bridge_candidate.yaml`，新增 `/camera/debug/points` 的最多8000点/1 Hz旁路；导航仍在域31使用完整点云。点云/体素颜色区分见[手册31.14](../../docs/操作手册.md#3114-打开原生深度点云预览)，当前完整操作用31.16，原15刷新配置保留作历史。

STVL 深度导航实验主线使用 `navigation_depth_stvl.launch.xml`；只看静止地图使用 `depth_costmap_stvl_static.launch.xml`。原生 VoxelLayer 和底层测试入口均保留。

避障输入使用原生深度点云经现有过滤后的障碍观测，保留宽视场；RGB配准图仅供实物对照，不作为STVL点云来源。三窗视场差异、黑色缺测与当前参数口径统一见[手册31.11](../../docs/操作手册.md#3111-深度配准到-rgb-的显示参照nav-016-r2)，不因RGB画面较窄调整安装TF或删除侧边障碍。

当前静止显示候选增加官方`domain_bridge`0.5.0单向域31→32桥，仅传预览/二维/TF/车模；相机和STVL参数不变，桌面不再直接加入车上域。独立XML`stvl_display_bridge_candidate.launch.xml`、YAML和域32发现配置为实验资产，原域31/2000点失败对照保留；依赖来源、固定版本/许可证、完整部署及停止恢复统一见[手册31.8](../../docs/操作手册.md#318-stvl-独立显示域与稠密预览nav-016-r2)。

完整构建、启动、观察、停止和恢复步骤只维护在[操作手册第31节](../../docs/操作手册.md#31-stvl-深度导航主线nav-016)。版本来源与许可证见[接入依据](../../docs/archive/D435_STVL接入依据_20261006.md)，离线及实机结果统一登记NAV-016。

## 接口与边界

- `carcar_navigation::MonitoredStvlLayer` 动态链接固定2.3.4上游库；只包装本轮输入状态、地图合并序号和耗时，没有实现新标记/衰减算法。上游插件头文件随固定版本保留，来源及LGPL全文在 `vendor/stvl_2_3_4`。
- 层名为 `depth_stvl`，诊断为父节点命名空间下的 `depth_stvl/diagnostics`，类型 `DiagnosticArray`。`source_current`、`update_seq`、`update_duration_ms` 取自本轮输入检查及地图更新；不以三维显示点云的发布时间判断实际消费。
- `depth_layer_supervisor` 的 `required_stvl` 模式持续发布 `/navigation/perception_motion_allowed`。启动/故障为false，过滤健康、推进的采样时间、两图有效输入/新更新连续健康3秒并各更新至少两轮后为true。不会切为仅激光继续运动，也不周期清图。
- 行为树通过 `recovery.require_perception_health`（默认false，STVL入口true）进入独立感知等待：取消控制动作、确认停稳、保留有效导航任务，不消耗普通输入/运动恢复预算；恢复后重新规划，已取消任务不重发，取消或停稳确认失败仍锁停。
- 速度联锁增加 `require_perception_health`（默认false，STVL入口true）、`perception_timeout`（默认0.30秒，上限0.30）。启用后必须收到持续允许心跳；关闭/失联清除旧授权及速度，重新开放后须收到新的BT授权与速度。
- STVL点云类型为 `PointCloud2`，地图命名空间下 `voxel_grid`；使用专用保存的RViz配置，不接旧VoxelGrid转换器。
- 线性时间衰减只表示旧观测过期。遮挡、视野盲区和无有效深度不是空闲证明；外参正确性、低障碍探测与实车制动仍须现场验收。
- 当前配置是实验默认值。二维导航网格5厘米、三维体素2厘米；小障碍能否产生可靠观测须实测，不能由网格尺寸推断。

静止实测前复核见NAV-016-R1：导航运行期确认后台执行器已启动后才允许对象退出，生命周期压力组连续200轮构造/销毁。主机显示同时启用过滤点、STVL体素及二维代价地图，`fastdds_rviz_wifi.xml`沿用项目单播发现设置（域31、Jetson 192.168.5.11）；主机实际地址和跨机显示仍须现场确认。

2026-10-08新增 `rviz/depth_costmap_stvl_equal_image_panels_candidate.rviz`：复用RViz/Qt布局保存，将原生深度、RGB配准深度、RGB三个窗口在底部等大同排，保留原候选和STVL显示参数。构建、Hao启动、观察、停止/恢复只见[手册31.12](../../docs/操作手册.md#3112-三个图像窗口等大显示)，验证登记NAV-016-R2，官方依据见[显示比例与配准调研](../../docs/archive/2026-10-08-D435图像显示比例与配准依据.md)。没有新生产执行逻辑。
