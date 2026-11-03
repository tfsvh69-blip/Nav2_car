# 自动建图稳定性优化与参数部署

<!-- NAV-018-STABILITY-001-CURRENT:BEGIN -->
## 2026-11-02 22:28 最终发布：r2默认软件已发布，现场验收待进行

本轮正式发布已成功。审核目录为 `log/nav018_stability_20261102/deployment-20261102-r2`；`publication.json` 存在，`failure.json` 不存在。共应用 278 项文件，保留 360 项原有发行文件，核对 15 个受测 ELF Build ID。正式服务 unit 未更改，地图仍指向主工作区原有动态资源；发布前完整备份已留在 `/home/jetson/luhao/my_nav_carcar_return_nearby_release.20261102-222759-418172.backup`。

r2默认配置的控制器全负载（path_aware_ranking=false） `final-20261102/controller-full-load-v6/summary.json` 通过：真实 Nav2/SLAM + 模拟轮式/激光 + 日志、快照、两个 Chromium 只读页面及实际 WebSocket 遥测/控制连接并发（两端共接收1974帧，未发送使能或运动请求），100% 覆盖、无接触、7 个快照、模拟路程 3.8416 m。控制器计算样本 521、P99 上界 1.25 ms、最大 1.401614 ms，无超过 100 ms 或异常；`/scan` 录入 1713 条。严格 TF 回归 `strict-tf-regression.xml`、雷达录包 QoS `raw-scan-qos-regression.xml`、监督器 `health-supervisor-serial.xml`、探索 `latest-explore-regression-180s.xml` 及排序接口 `latest-ranker-regression.xml` 均有通过记录。此前阶段未通过的长负载及对称场景证据保留，不能当作已修复的实车问题。

发布后独立 `deploy.py --check` 通过，四个包解析到正式目录，运行网关及网页三项静态资源校验一致。Windows 台式机访问 `http://192.168.5.11:8080/` 为 HTTP 200，`/api/status` 为 `STANDBY/manual`，没有任务、故障或未保存地图。**没有启动实车硬件、没有远程运动验收。** 本轮自动短退、连续探索和实验路径排序均默认关闭；0.22 m 硬净空、0.30 s 底盘看门狗和常用 0.10 m/s 保留。

历史旧 underlay 中字节不变的 `libcarcar_monitored_stvl.so` 有一项既存缺库 `libspatio_temporal_voxel_layer__rosidl_typesupport_cpp.so`，本轮冻结比对未引入新的缺库；不代表旧 STVL 深度层已修复。见 `deployment-20261102-r2/BASELINE_DEPENDENCY_AUDIT.md`。

后续实车静止及低速短测由现场操作者发起，重点为实际轮速、制动、近墙、窄通道、真实激光盲区、探索返程，以及此前独立 0.343 秒底盘看门狗事件。`--apply` 已成功执行，不得重复；只读复查用 `deploy.py --check`，如需回退须先确认任务结束、停车及待命，再按操作手册第 60.6 节执行 `--rollback`。
**实验路径排序未采用、未启用：** 对称探索v6的ranked达到100%真值覆盖但耗尽240秒未完成；v7虽15项回归通过，单轮legacy/ranked均240秒budget，完整场景仍未通过。正式r2继续path_aware_ranking=false；v7仅保留候选代码和失败证据，本轮停止继续调参，24矩阵未启动。
**正式Windows页面验证已完成（2026-11-03）：** 真实Edge的1440×1000桌面视口和390×844移动模拟视口均为STANDBY/manual，页面HTTP 200，无JavaScript运行异常、横向溢出或按钮遮挡；每页建立1次真实WebSocket握手并收到欢迎帧，只发送transport/client_event。移动尺寸是在Windows Edge中模拟，**不是手机实机验收**；未触发硬件或运动。 证据见[验证数据](../../../log/nav018_stability_20261102/final-20261102/windows-formal-20261103/verification.json)与[截图复核](../../../log/nav018_stability_20261102/final-20261102/windows-formal-20261103/review-and-manifest.json)。

**11月3日最终交接：** 约02:22的r2只读复检再次通过，检查对象为publication绑定的冻结安装，不代表当前stability候选已发布。02:31:50正式STANDBY/manual、busy/dirty=false、error/task_id为空；排名、连续探索和短退仍false，隔离测试进程已清理。候选stability目录包含未采用的v7，与正式r2须按各自指纹区分。见[最终状态](../../../log/nav018_stability_20261102/final-20261102/final-handoff-20261103-023150-092244.json)及[v7最终决定](../../../log/nav018_stability_20261102/frontier-v7-20261103/final-decision.json)。

<!-- NAV-018-STABILITY-001-CURRENT:END -->

测试编号：NAV-018-STABILITY-001  
日期：2026-11-02  
候选研发目录（现保留未采用v7，r2源以冻结清单为准）：/home/jetson/luhao/my_nav_carcar_stability_20261102  
正式发布目录：/home/jetson/luhao/my_nav_carcar_return_nearby_release

## 发布前交付状态（历史记录）

**以下为历史发布前阶段性记录；当前版本状态以本页顶部最终发布记录为准。**

普通8080在正式发布前是旧发布的待命状态，无活动硬件任务。候选中的软件运动积分、模拟扫描和真实Nav2/SLAM节点均在本机隔离ROS域运行；它们不代表物理车辆行驶或实车验收。

用户此前已经批准全部问题的实现及验证后更新8080。实车上电与运动验收仍由现场操作者发起；自动短退只交付候选，正式开关保持关闭。

## 问题、修改与作用

| 问题 | 实施内容 | 预期作用与边界 |
|---|---|---|
| 雷达超时后网页轮式缓存也停止更新 | 写图/运动禁止与只读观测分离 | 故障时继续显示真实传感器年龄；输入恢复不自动放开写图与运动 |
| 定位失败同时封死人工接管 | 冻结地图下独立检查低速接管资格 | 取消自动目标、确认停稳与新鲜扫描后才能接管；松手、断线和盲区仍停车 |
| 恢复验证再次触发错误回环 | 冻结参考图匹配，保留拒绝约束与验证条件 | 正常建图的回环仍保留，不能把参考匹配成功等同于任意故障均可恢复 |
| 同一路径反复转正、前进置零 | MappingRotationShim与RPP分工 | 大角度先对准，小角差由路径跟踪修正；保留短路径、末端朝向与碰撞检查 |
| 路径和终点贴边 | 扩大软代价范围，增加规划代价权重，观测点使用有上限的净空偏好 | 硬净空与未知区拒绝不变；更居中的路径可能略长 |
| 短时阻挡即要求人工继续 | 有界等待、重新观测与重新规划 | 无法证明运动安全时仍保持停车；不会通过正常导航倒车规避阻挡 |
| 选点只看直距、前沿已看清仍走到底 | 真实规划路程排序、可见未知收益、转向/重复访问成本、前沿失效取消 | 取消须等待终态，有效目标继续保持，不能用测试场景的真值覆盖率决定生产结束 |

## r2已发布参数与保留关闭项

下表记录r2正式参数及关闭项，不是未采用v7候选的发布声明；数值须连同正式普通入口的加载关系核对，不能只检查候选YAML是否存在。

| 参数/功能 | r2发布前 | r2发布配置/关闭项 |
|---|---:|---:|
| 全局 inflation_radius | 0.30 m | 0.50 m |
| 全局 cost_scaling_factor | 3.0 | 3.0 |
| GridBased.cost_travel_multiplier | 1.0 | 2.0 |
| ExplorationGrid.cost_travel_multiplier | 1.0 | 2.0 |
| 局部 inflation_radius | 0.20 m | 0.30 m |
| 局部 cost_scaling_factor | 5.0 | 5.0 |
| RPP cost_scaling_dist | 0.30 m | 0.30 m |
| RPP inflation_cost_scaling_factor | 5.0 | 5.0 |
| 控制器 | RPP自身反复判断转正 | MappingRotationShim + RPP |
| Shim进入/退出转正角差 | 无独立模块 | 0.50 / 0.10 rad |
| RPP自身 use_rotate_to_heading | true | false，由Shim负责 |
| 前视范围 | 0.20～0.30 m | 保持 |
| 默认前进档 | 0.10 m/s | 保持 |
| 受限接管上限 | 依赖完整定位健康 | 0.05 m/s、0.10 rad/s，另验方向与冻结状态 |
| 硬路径净空 | 0.22 m及既有规划掩膜 | 保持 |
| 底盘命令超时保护 | 0.30 s | 保持 |
| 自动短退 | 未启用 | false，待独立现场验收 |

启动接线检查发现：原候选虽有新文件，普通Launch仍默认旧RPP/旧软代价，多个新功能开关也仍为false。本轮将父级mapping_autonomous与navigation两层入口一起修正。新任务须实际加载controller_rpp基线后再叠加controller_shim_rpp；软代价使用nav2_clearance。本轮v6/v7完整场景均未满足排序采用条件，正式路径排序保持false，历史连续多目标模式不顺带开启。

## 已有量化证据

### 离墙偏好与窄通道

两份实车录制地图的静态重建中，旧软场和现有硬规划掩膜叠加后，可通行格内非零软代价为0；扩大全局半径后约37%的可通行格有软梯度，可通行格总数不变。历史真实运行代价图另有约3.4%的软代价格，不能把静态重建结论扩大成现场始终完全没有梯度。

两条重规划路径的参考点到已知障碍中位距离分别由0.34765增至0.47953 m、0.44299增至0.59436 m。第一条路径由2.0939增至2.38695 m，第二条由4.1281变为4.0804 m。0.65、0.75、0.90 m三种通道宽度中两个规划器仍可通过。

局部0.20/0.30/0.35 m对照可完成，但没有证据证明单独扩大局部膨胀明显缩短耗时。选择0.30 m主要是与RPP的0.30 m近障减速距离一致，并保持最小必要改动。

### 控制器选择

此前同条件99次软件闭环：RPP 33/33完成，Shim+RPP 33/33完成，MPPI 30/33完成；MPPI短返程在三个固定种子下均进展超时，不能进入本次正式默认。

历史长路径样本中，Shim+RPP把两次反复走停降为0，59.2秒降至53.8秒。这是软件控制器对照，不能当成实车效果或完整负载实时性结果。

### 最终故障与几何回归

本轮重跑137项C++检查和2组JavaScript检查，均通过。包括26项操作恢复、23项真实网关集成、受限运动方向、原始激光缺束、取消与迟到反馈、冻结地图、共享操作及网页按钮状态。

证据：log/nav018_stability_20261102/final-20261102/fault-and-geometry-summary.json。

探索客户端新增前沿失效取消、取消拒绝、坐标世代、暂停迟到结果等边界，23项接口回归通过。证据：log/nav018_stability_20261102/frontier-observed-cancel-tests.xml。完整场景结果另记，不能用这些接口检查代替覆盖与效率验收。

### 完整负载首轮反例

首轮同时运行SLAM、快照、网页、录包时，控制计时累计522次，P99上界102.25ms、最大2502.63ms、7次超过100ms，未通过原门槛。最长一次发生于退出等待；运行中仍有6次约100～136ms，不能通过排除退出样本掩盖。

源码确认Humble 1.1.20 Shim将采样点时间改成now，可能等待尚未到达的未来TF。正在局部适配为本次有效控制位姿对应的时间，并保留原新鲜度、碰撞、限加速度和朝向检查；新鲜TF下不得以增加异常或停车来换取较小计时。

另发现记录器提前订阅时把两个后出现的BEST_EFFORT话题按RELIABLE处理，导致controller_timing及slam_scan漏录；补明确QoS并验证实际录包消息。失败证据保留在final-20261102/controller-full-load，修复后须使用新目录复验。

## 发布与地图兼容

正式unit保持原加载顺序。候选安装目录不是完整colcon overlay，正式发布必须包含新Shim的库、插件描述、ament索引、参数和Launch；新explore/logger包的引导文件也必须完整。

carcar_navigation中的动态maps资源继续连接主安装目录的同一地图视图，不冻结一份会遮蔽以后地图更新的旧副本。部署只修改应用文件和该连接，不修改主地图、录包或历史失败证据；回退不撤销以后正常审核的地图更新。

维护脚本和待审清单位于log/nav018_stability_20261102/deployment-20261102-r2。构建、核验、部署和回退的完整有效命令在[操作手册](../../操作手册.md)统一维护。

## 尚待现场确认

软件完成、正式部署与实车验收分别登记。实车仍需验证实际轮速、制动、真实扫描盲区、短程探索与返程、墙边和狭窄通道表现。

本轮0.343秒底盘看门狗事件，已有证据指向驱动接收/调度环节：最终速度持续约50ms发布，但轮式与驱动诊断同时间断约0.35s。尚无足够进程级证据确认具体阻塞来源，不宣称该底盘问题已修复，0.30秒保护保持。

## 参考

- [Nav2 Humble Smac Planner](https://api.nav2.org/nav2-humble/html/md_nav2_smac_planner_README.html)
- [Nav2 Humble RPP](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html)
- [Nav2 1.1.20 Rotation Shim源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)


## 11月2日晚间最终候选复验与录包修正

- 为避免误把离线仿真说成实车验证，本轮全部单独使用 ROS_DOMAIN_ID=200～232、ROS_LOCALHOST_ONLY=1、hardware=false。正式8080直到发布脚本成功返回前保持旧发布待命。
- 双 Chromium（桌面/移动）只读页面预检 V4 已通过，见 final-20261102/controller-browser-preflight-v4/summary.json。两边 WebSocket 均建立且接收到欢迎帧，页面状态持续更新；测试不发运动请求。
- 最新与原始目标相同的控制器在四场景选定的软件回归中为 6/6，首轮并发测试曾显示等待未来TF的约百毫秒卡顿；按Nav2控制位姿时刻对齐后，V4并发闭环完成，100%覆盖、8次快照、控制计算P99为2ms、最大4.458ms、无计算超过100ms或异常。两端浏览器持续接收数据，路程约4.40m；见 final-20261102/controller-full-load-v4/summary.json。
- **V4整体仍不通过**：真实录包中 /scan 的消息为0（但 /scan/filtered≈1802、/mapping/slam_scan≈1798、轮式≈1851）。已从运行日志找到明确的 RELIABILITY_QOS_POLICY 不兼容，原始雷达发布为 BEST_EFFORT，提前启动的录包对 /scan 却错误请求 RELIABLE。修复为 /scan 固定 SensorDataQoS，新增发布者先于/晚于录包两种顺序的原始雷达读包断言，C++回归已通过；历史V4失败结果原样保留，最终并发负载另起新证据目录，不覆盖V4。
- 路径排序最新对称场景V5：旧逐点 15.4628m、排序候选10.4438m，两者都是100%真值覆盖，但**均耗尽240秒，没有正常结束**。因此不能把该局部约32.5%路程缩短和约48.1%单位路程收益计为整个连续探索门槛通过；正式 path_aware_ranking=false，连续探索和自动短退仍关闭。源代码和失败证据保留作下一轮分项研究。
- 最终发布只能依据重编译、录包回归、真实网页并发复测及完整白名单/地图/ELF一致性检查。部署和回退的唯一命令入口为操作手册第60节。


## 11月3日固定版本 v6 对称探索：排序候选未通过

本次为隔离 ROS 域、hardware=false 的车载软件闭环；两个分支使用相同0.10m/s速度、控制器与软代价配置，预算均为240秒。这里的“对称探索v6”与顶部已通过的“控制器全负载v6”是不同测试，不混用结论。

| 分支 | 终态 | 探索耗时 | 路程 | 真值覆盖 | 接触 |
|---|---|---:|---:|---:|---|
| legacy | complete | 194.3988 s | 12.56485 m | 100% | 无 |
| ranked | budget | 240.0010 s | 12.64753 m | 100% | 无 |

**ranked虽然达到100%真值覆盖，仍耗尽预算而未正常完成，完整场景验收失败；本轮也没有路程改善。** 真值覆盖只用于离线评价，不作为生产完成条件。原始结果分别见[legacy结果](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-legacy/result.json)、[ranked结果](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-ranked/result.json)及[运行前指纹](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-preflight.json)。v2、v5及本次v6失败均保留。

### 转向差额与终点朝向假说

原地转向按EXPLORING阶段、abs(vx)≤0.02m/s、abs(wz)>0.02rad/s，对相邻命令的左区间积分。ranked为75.00017秒，legacy为50.34891秒，额外24.65126秒。ranked前7个短观测点的5个起步转向块合计35.39958秒，legacy前3个观测点合计10.94776秒，局部阶段多24.45182秒；后续3次长支路起步仅多0.19944秒。

ranked全部8个原地转向块发生在新目标起步，未观察到到达0.15m位置容差后持续对齐终点yaw、再反向原地转去下一点。探索BT本就绑定PositionGoalChecker，只验收XY位置；该配置与pair预检、r2发布及候选/正式安装SHA一致。ranker仅输出观测点XY，supervisor按预检路径末端至少0.10m切线设置探索目标yaw；360°unknown gain不读取yaw。**本次额外转向来自局部短点序列的多次起步，终点yaw假说已由实际记录与绑定证据排除；不据此调整全局yaw容差或返程朝向。** trace约1Hz，审计未声称拥有未记录的精确目标四元数或规划末段角差。

完整只读审计已新增保存12个文件，见[yaw审计](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-yaw-audit/README.md)、[8个转向块与口径](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-yaw-audit/nav_yaw_summary.json)、[7个短点表](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-yaw-audit/seven-short-goals.json)、[源码/BT绑定](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-yaw-audit/source-fingerprints-and-binding.json)与[SHA清单](../../../log/nav018_stability_20261102/exploration-symmetric-v6-final-20261103-yaw-audit/manifest.json)。

**正式r2仍为path_aware_ranking=false，连续探索与自动短退仍关闭。后续v7单轮对称两模式均未完成，最终不采用、不启用，详见下方收口记录。** 不重复r2 apply，不覆盖失败、原地图或备份。

## 11月3日正式监督器健康探测边界补验：3/3通过

使用正式发行的mapping_autonomy二进制（SHA256 `134bd636f1f34edb8be5682b76030ebd11d0675074f8615b346b8b47110f58bc`），在隔离ROS域206执行真实进程集成测试；没有启动车辆硬件或修改生产逻辑。三个用例均通过，构建与测试退出码均为0。

| 边界 | 实际结果与含义 |
|---|---|
| 单次健康请求超过1秒 | 请求可超时并重试，但最近一次明确ACTIVE答复仍在2秒有效期内时，不提前锁存停车；本次观测答复年龄1.754661秒。 |
| 最后一次ACTIVE答复超过2秒且后续请求无答复 | 失去健康资格，锁存FAULT并拒绝速度；本次故障时答复年龄2.152980秒。 |
| 服务明确答复inactive | 拒绝继续自动行驶，不能用旧ACTIVE答复抵消明确inactive。 |

**1秒是单次请求超时/重试边界，2秒是最后一次有效ACTIVE答复的保鲜边界；不应写成“服务1秒无回复就停车”。** 准备与结束记录中的6个受监测生产源码、构建产物及正式运行文件SHA逐项相同，正式安装没有被测试构建替换。域206残留进程为空，测试已清理。

证据：[summary](../../../log/nav018_stability_20261102/final-20261102/health-probe-boundary-20261103/summary.json)、[精确边界与清理核验](../../../log/nav018_stability_20261102/final-20261102/health-probe-boundary-20261103/verification.json)、[GoogleTest结果](../../../log/nav018_stability_20261102/final-20261102/health-probe-boundary-20261103/results.xml)、[实际正式二进制执行记录](../../../log/nav018_stability_20261102/final-20261102/health-probe-boundary-20261103/test-process.json)。这是既有r2行为的边界补证，不代表v7候选已通过。


## 11月3日v7候选最终收口：单轮未完成，不采用、不启用

v7研究仅发生在`/home/jetson/luhao/my_nav_carcar_stability_20261102`。候选将固定目标切换成本从0.15m调至0.35m，使局部候选遵守实际score次序，并增加默认关闭、最多256条的prefix诊断；仅ranked测试开启诊断。原速度、净空、gain算法、控制器、真实前沿完成判据及返程朝向没有为测试放宽。正式r2不包含这次候选修改。

13项frontier-gain与2项frontier-contract回归均通过，静态审查无阻断；这15项回归只证明对应功能和接口，不能替代完整任务完成或效率验收。随后仅运行一轮对称legacy/ranked对照，使用0.10m/s、240秒预算及相同控制器/软代价：

| v7本轮模式 | 终态 | 探索耗时 | 路程 | 单位路程收益 | 覆盖 | 接触/快照 |
|---|---|---:|---:|---:|---:|---|
| legacy | budget，退出1 | 240.0010 s | 14.75570 m | 0.081494 m²/m | 100% | 无 / 24 |
| ranked | budget，退出1 | 240.0012 s | 13.20246 m | 0.091082 m²/m | 100% | 无 / 24 |

ranked比本轮legacy局部路程少10.5%、单位路程收益高11.8%，但两边均未真实complete，**不得解释为已验收收益或采用资格**。v6的legacy曾以194.3988秒/12.56485m完成，本轮legacy也耗尽预算，存在尚未单独查明的运行间差异；legacy不使用v7评分，不能把该差异归因于v7评分修改。

运行前后121项输入指纹一致，实际ranker二进制与Shim库的加载指纹绑定成立；域231/232进程已清理，测试端口释放，正式8080保持STANDBY/manual。候选代码、补丁前备份、候选安装前副本、审查、构建、回归、失败结果与有界诊断完整保留。

**最终决定：本轮不采用、不启用v7路径排序，停止继续调参，不执行24轮矩阵。** 正式继续使用r2冻结发行，path_aware_ranking=false；候选工作区的当前源码/构建不再等同正式r2。未来如重新设计排序，须另建受测快照和证据，不能隐式恢复本轮未通过策略。

证据：[15项回归](../../../log/nav018_stability_20261102/frontier-v7-20261103/build-finished.json)、[单轮汇总](../../../log/nav018_stability_20261102/frontier-v7-20261103/pair-summary.json)、[指纹/清理结果](../../../log/nav018_stability_20261102/frontier-v7-20261103/pair-finished.json)、[实际加载绑定](../../../log/nav018_stability_20261102/frontier-v7-20261103/loaded-runtime.json)、[最终决定与完整证据SHA](../../../log/nav018_stability_20261102/frontier-v7-20261103/final-decision.json)。

### 正式r2最终状态与现场边界

11月3日约02:22，r2 deploy.py --check再次退出0，正式网关PID23759及静态资源与冻结发行匹配；这是正式安装/状态校验，不代表v7已发布。02:31:50最终读回STANDBY/manual、busy/dirty=false、error/task_id为空；正式7项开关中排名/连续/短退为false，净空偏好/受限人工/执行扫掠/有界等待为true。域206/225/231/232没有遗留测试进程，也没有继续参数扫测，见[最终交接证据](../../../log/nav018_stability_20261102/final-20261102/final-handoff-20261103-023150-092244.json)。

已经完成的正式Windows页面与健康3例证据保留各自范围：移动仅Windows模拟视口；健康请求1秒重试与最后ACTIVE答复2秒有效期分别计时，明确inactive仍拒绝。实车轮速、制动、近墙、真实盲区及此前独立0.343秒底盘看门狗事件仍需现场证据；后续实车任务由操作者发起。原地图、r2完整备份及回退守卫保留，完整命令仍只维护于操作手册60节。
