# 仓库指南

## 项目结构与模块组织

本仓库是一个独立的 ROS 2 Jazzy 巡检小车工作空间。

- `src/inspection_bringup/`：本项目的启动文件、RViz 配置和系统集成入口。
- `src/sllidar_ros2/`：引入的 SLAMTEC 上游驱动；修改上游代码时必须单独记录原因。
- `scripts/`：面向使用者的构建、硬件检查和启动脚本。
- `docs/硬件实测记录.md`：用户确认的硬件能力、限制、测试条件和变更历史。
- `dependencies.repos`：固定外部源码版本，便于复现环境。
- `build/`、`install/`、`log/`：colcon 生成目录，不得手工修改或提交。

新增功能应拆分为职责明确的 ROS 包，例如 `inspection_description`、`inspection_base`、`inspection_perception` 和 `inspection_navigation`。硬件组合统一放在 `inspection_bringup`，避免驱动之间直接耦合。

## 构建、测试与开发命令

在仓库根目录执行：

```bash
./scripts/check_lidar.sh       # 检查 USB 串口和访问权限
./scripts/build.sh             # 使用 --symlink-install 构建全部包
./scripts/run_lidar.sh         # 启动 /dev/ttyUSB0、静态 TF 和 RViz2
./scripts/run_lidar.sh /dev/ttyUSB1
```

直接使用 ROS 命令前加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

使用 `ros2 topic hz /scan` 和 `ros2 topic echo /scan --once` 完成雷达冒烟测试。

## 编码风格与命名约定

Python 使用 4 空格缩进，XML/YAML 使用 2 空格缩进。Python launch 文件遵循 PEP 8，CMake/C++ 遵循 ROS 2 常用规范。包、节点、话题、参数、脚本和 launch 文件使用 `snake_case`，例如 `lidar_test.launch.py`。坐标系使用 `base_link`、`laser` 这类小写 ROS 标识符。

launch 参数必须可配置，不要在代码中写死机器专属设备名，文档化的默认值除外。Shell 脚本使用 Bash，并始终引用变量。

## 文档语言规范

仓库内新增或修改的 README、贡献指南、配置说明、用户提示和面向维护者的注释必须使用简体中文。命令、代码、路径、ROS 接口名、专有名词和第三方许可证保留原文；必要时可在中文说明后附英文术语。引入第三方英文文档时，应提供中文版本或中文摘要。

## 实测反馈维护规范

用户明确确认的硬件现象、稳定工作范围、参数结论、故障原因或兼容性信息，必须在同一次任务中同步更新 `docs/硬件实测记录.md`。记录应包含日期、模块、结论、测试条件、信息来源，以及可复现该结论的命令；条件未知时写“待补充”，不得自行推测。实测过程中确认有效的检查、启动、诊断命令也必须更新到文档的“常用命令速查”。新结论与旧记录冲突时不得直接覆盖，应保留历史并注明新结论取代旧结论的原因。未经实体测试的信息标记为“软件配置”或“待验证”，不能写成实测结论。

## 测试规范

当前没有自动化测试套件。新包应加入 `ament_lint_auto` 及适当的测试工具（`ament_cmake_pytest` 或 `ament_cmake_gtest`）。测试命名为 `test_<行为>.py` 或 `test_<组件>.cpp`。提交前必须构建整个工作空间、运行 `colcon test`，并完成相关硬件冒烟测试。单元测试不得依赖实体硬件。

## 提交与合并请求规范

当前没有可用的根仓库 Git 历史。提交信息使用简短的祈使句并可带范围，例如 `bringup: 添加可配置的雷达变换`。生成目录不得进入提交；上游驱动修改和本项目集成修改应分开提交。

合并请求必须说明行为变化、列出验证命令、注明测试硬件和 ROS 发行版。涉及可视化时附 RViz 截图；关联相关问题，并明确新增的设备权限、udev 规则或系统依赖。

## 安全与配置

不要使用 `sudo` 运行 ROS 或 RViz。通过 `dialout` 用户组授予串口权限。不得向用途不明的雷达线缆施加外部电压；设备端口、波特率、坐标变换和扫描模式必须暴露为 launch 参数。
