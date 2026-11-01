# 固定第三方依赖

NAV-018-R24／R25来源保留（2026-10-30，2026-11-01补记）：`frontier_ordering` 的 `frontier_types.hpp`、`mrtsp_ordering.hpp/.cpp`、`mrtsp_solver.hpp/.cpp` 原样取自 [mertgulerx/frontier_exploration_ros2 固定提交11864e4ac6789dc81fd71bad51e43339c01deca2](https://github.com/mertgulerx/frontier_exploration_ros2/tree/11864e4ac6789dc81fd71bad51e43339c01deca2)，Copyright 2026 Mert Güler，Apache-2.0。文件版权声明和完整LICENSE保留。2026-11-01联网逐文件比对，5项源码／头文件及LICENSE均与该提交逐字节一致。曾供有限前瞻排序实验；R25已从默认编译依赖中移除，现保留历史源码和许可证，不代表正式导航启用该排序器，不引入上游整包或替换本车Smac路径审查。下方R10“未复制源码”保留其更早阶段的历史含义。

- explore_lite / explore_lite_msgs：robo-friends/m-explore-ros2，commit `326cf8a0b487c34246bb8f3326afbcd69576dc60`，BSD-3-Clause。只引入单车探索，不引入多机合图。原许可证及源文件归属保留。
- laser_filters：ros-perception/laser_filters，tag `2.0.10`，BSD。仅使用 Speckle Distance (`filter_type=0`)。RadiusOutlier 的索引检查顺序在本地修正，仍不作为本车默认模式；另修正 validator 析构条件并补齐基类虚析构，保证插件销毁与重配置安全释放。

本项目补丁：探索冷待命、只取消当前持有目标、发送中取消、结果代次检查、黑名单耗尽单独终态、结束后释放GoalHandle；增加显式 `navigation_action` 参数，避免依靠action基名重映射。目标使用距机器人最近的真实前沿栅格 `middle`，避免连通前沿的均值落回车旁；仍经监督器投影和路径审查。上游探索状态只作输入，不作为停车或完整建图证明。上游原始 Python Launch 保留为第三方来源，不作为本项目操作入口；本项目入口为 XML。

固定来源：https://github.com/robo-friends/m-explore-ros2/tree/326cf8a0b487c34246bb8f3326afbcd69576dc60 ，https://github.com/ros-perception/laser_filters/tree/2.0.10 。未升级系统ROS包。

另见兄弟包 [carcar_tf2_compat](../carcar_tf2_compat/README.md)：geometry2 0.25.23固定commit的局部锁序修复，仅新XML子进程显式加载私有库，不替换系统TF2或改变旧入口。来源、ABI范围、许可证与退出条件在该包记录。

NAV-018-R10补丁（2026-10-28）：探索活动目标进展绑定实际Nav2反馈；暂停／恢复保留有限区域记忆，成功等待新图，同簇代表点回退。区域等价0.40米、60秒冷却、2次失败暂避、0.10米进展均为本车实验候选。仅借鉴 frontier_exploration_ros2 固定11864e4ac6789dc81fd71bad51e43339c01deca2的dispatch与抑制设计，未复制其源码、参数或引入依赖。新增真实Explore节点Action生命周期测试，原历史判点测试保留。详见[切点诊断](../../docs/archive/2026-10-28-自动建图切点诊断/README.md)。

NAV-018-R14补丁（2026-10-29）：原固定Explore来源不升级，BSD-3-Clause声明保留。显式配置的候选拒绝话题传递本车监督器确认“未提交路径预检”的proxy UUID；拒绝与ABORTED终态均匹配后，一次100毫秒定时重选，处理消息乱序、旧UUID和暂停取消，不提高周期频率或改区域记忆。该信号及调度为本项目适配，并非上游既有功能。参考上游固定版本[FrontierSearch](https://github.com/robo-friends/m-explore-ros2/blob/326cf8a0b487c34246bb8f3326afbcd69576dc60/explore/src/frontier_search.cpp)的连通区域搜索与[Humble SmacPlanner2D](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_2d.cpp)的实际代价图规划边界，自研静态观测点连通筛选；未复制上游源码、添加生产依赖或修改系统库。替代方案为提高全局探索频率或重做前沿搜索地图，本轮不采用；边界见本轮测试与手册39.9。

同轮局部适配来源与边界：

- 自动建图行为树参考 [Nav2 Humble周期重规划树](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)，实际使用本机Nav2 1.1.20／BehaviorTree.CPP 3.8.7既有插件（Apache-2.0／MIT）。规划失败仅Delay 400毫秒后再试一次同一目标，保留FollowPath失败终止；未启用清图、Spin、BackUp或Wait动作。其他导航入口的无恢复树保留。
- 观测点到前沿的可见性使用自行实现的二维栅格DDA遍历，思路参考 [Amanatides／Woo 1987原论文](https://www.eecs.yorku.ca/~amana/research/grid.pdf)。遍历包括角点两侧格，未知只允许位于前沿终点，不复制第三方代码、不新增依赖。
- Nav2反馈的语义核对参照 [Humble NavigateToPoseNavigator](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp)：其onLoop按共享path计算距离，跨目标开始时可短暂带上旧值。本车按固定实际观测点的直线下界加候选余量拒绝明显偏短反馈，不替换Nav2、并发规划或复制上游实现。RPP内部角加速度对照参考同版本[官方控制器](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)的默认3.2及实测速度限幅机制；最终平滑器及速度限制保留，候选实车结果单列。
- NAV-018-R12按[Humble VelocitySmoother](https://api.nav2.org/nav2-humble/html/velocity__smoother_8cpp_source.html)既有同步缩放机制，仅在建图专用配置启用候选，不复制实现。本机Nav2 1.1.20／Apache-2.0的Smac、RPP、Costmap及VelocitySmoother另用于BUILD_TESTING内的真实快照回放／加减速回归，不新增系统安装；测试节点域209、速度输出重映射隔离，快照工具不安装为车端节点。原普通导航false、上限与停车门槛保留，候选实车收益待验。
- `mapping_slam`栅格边界修正依据本机slam_toolbox **2.6.10** 的 [Karto OccupancyGrid](https://github.com/SteveMacenski/slam_toolbox/blob/humble/lib/karto_sdk/include/karto_sdk/Karto.h) 与 [SMapper接口](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_mapper.cpp)。Karto SDK保留LGPLv3声明；slam_toolbox包声明LGPL，上游SMapper源文件另有CCPL声明。项目新增子类调用原`CreateFromScans`，仅闭区间尺寸增加一格以容纳最大实测端点；原射线计数、占用阈值和优化扫描不变。通过本可执行文件内同签名`SMapper::getOccupancyGrid`调用适配，精确版本检查约束ABI；未复制整段上游实现、替换系统库或使用LD_PRELOAD。其他SLAM入口不变。版本升级需重新核对符号和真实进程回归，不将其视为通用上游补丁；替代方案为上游修复后移除此适配。测试时本机库与头文件SHA256保存在本轮证据目录`compatibility-dependency-sha256.txt`。
