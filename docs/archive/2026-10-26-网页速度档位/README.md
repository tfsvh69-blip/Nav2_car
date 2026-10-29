# 2026-10-26 网页速度档位证据

用户要求提高速度上限、提供明显可调档位，由现场判断。当前结论统一[APP-001-R15](../../测试记录.md#app-001-r15)，完整命令与停止恢复统一[手册35.12](../../操作手册.md#app-001-r15-operation)。

## 实现与边界

- C++异步参数设置／读回，停车切档；四档0.10／0.18／0.25／0.30m/s，重启默认0.18。独立巡检包络0.30，最终输出按确认档位裁剪，保护超时不变。
- 原生接口来源固定为Nav2 1.1.20：[DWB动态参数](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/kinematic_parameters.cpp)（BSD），[RotationShim转发](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)（Apache-2.0）。只使用安装接口，没有复制源码；`min_speed_theta=0`同请求重申以刷新最大平移速度平方缓存。
- 对照过[SpeedLimit消息](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_msgs/msg/SpeedLimit.msg)及其控制器入口；DWB会按比例缩放角速度，本轮采用参数服务，只调整平移档并提供读回确认。
- [speed_profile_probe.cpp](speed_profile_probe.cpp)只操作速度配置及状态；无导航、定位、角度或速度发布。没有网页连接且目标IDLE／软件停稳时运行，五次切档成功后回0.18。

## 证据位置

原始日志保留在[本轮日志目录](../../../log/inspection_speed_profiles_20261026/)：网页构建／6项测试、目标包构建、首次浏览器状态更新时序失败、17项控制台重跑通过、总计54项巡检结果、部署前后状态、实机五次切档及限速读回；修改前源码／Launch同目录保留。

[实际组合启动日志](../../../log/inspection_goal_20261026_154154/robot.log)记录启动默认0.18、S1初始化和五次速度确认。MainPID18281，静止探针期间没有目标或非零速度。软件配置／读回通过不代表实物已达到对应速度、抖动已消失或停车距离已验收。
