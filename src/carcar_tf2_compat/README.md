# Humble TF2局部兼容库（实验）

来源：[geometry2 0.25.23](https://github.com/ros2/geometry2/tree/9bc9e89285515c8981ecd829c9400807372468c4)，固定commit `9bc9e89285515c8981ecd829c9400807372468c4`。仅引入tf2_ros共享库的7个源文件，完整BSD-3-Clause归属保留在各文件。使用本机0.25.23头文件，类布局及导出接口不变，不安装或覆盖系统库。

本机软件房间GDB记录确认MessageFilter注册线程与TF接收线程形成AB-BA等锁，对应上游[#992](https://github.com/ros2/geometry2/issues/992)与[#995](https://github.com/ros2/geometry2/issues/995)。这里是**本项目局部修复**，不是已合并的上游补丁，也不宣称ROS发行版认证。

修复 `Buffer::waitForTransform` 注册过程：在不持有timer映射锁时向BufferCore注册；如果TF在定时器登记前到达，用单次共享注册状态暂存通知，完成定时器关联后再执行。注册过程不持有通知状态锁调用BufferCore，避免反向锁序；定时器超时保留。`cancel`另按映射的request值查找timer并移除，避免误把request当timer键、留下迟到回调。既不回退到0.25.20的丢早到回调竞态，也不延长数据超时。

库安装在私有 `lib/carcar_tf2_compat/`，普通工作区环境不会自动选中；只有新自动建图／纯激光XML子进程显式设置自己的搜索路径。现有实车进程、旧入口和 `/opt/ros/humble` 未更改。上游正式修复后须重新跑并发、超时／取消和全软件房间回归，再移除局部兼容路径。
