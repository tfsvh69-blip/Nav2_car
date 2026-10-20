# 停用导航草稿归档

归档日期：2026-10-10。两份源码完整迁移，未删除内容、未恢复构建或启动。仅用于历史审计，**不得作为当前实车入口**。

| 文件 | 原路径 | 停用原因与替代关系 |
|---|---|---|
| [escape_controller.cpp](escape_controller.cpp) | `src/carcar_navigation/src/escape_controller.cpp` | 直接逃生速度缺少恢复预算及数据年龄保护；现使用应用于导航树的 `SafeBackUp` 与 `ProtectedBackUp` 受保护 Action |
| [wheel_imu_ekf_node.cpp](wheel_imu_ekf_node.cpp) | `src/carcar_navigation/src/wheel_imu_ekf_node.cpp` | 自写 EKF 缺少时间戳对齐、安装 TF、完整协方差及异常处理；未来仅在基础导航稳定后独立评估官方 `robot_localization`，不启动此草稿 |

当前 CMake 显式列出构建源码，未引用以上两文件；插件 XML、Launch、脚本和测试均无有效调用。迁移不改变当前运行入口。

配套 [escape_policy.hpp](../../../../src/carcar_navigation/include/carcar_navigation/escape_policy.hpp)、[wheel_imu_ekf.hpp](../../../../src/carcar_navigation/include/carcar_navigation/wheel_imu_ekf.hpp) 及 [test_nav_escape_logic.cpp](../../../../src/carcar_navigation/test/test_nav_escape_logic.cpp) 仍在原位，因为 CMake 注册的测试仍使用它们。节点草稿停用不等于算法测试资产无效。

迁移前后的 SHA256 与相关离线检查结果统一见[测试记录](../../../测试记录.md)。历史记录中的原路径表示当时位置，保留原陈述，另加归档修订说明。Git 历史可追溯原内容；只为阅读追溯时无需移回，回滚归档可按[操作手册第32节](../../操作专题原文_20261014.md#32-巡检设计与停用草稿归档的离线复核)操作。
