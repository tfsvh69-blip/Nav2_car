# 发行依赖基线例外审计（NAV-018-STABILITY-R2）

2026-11-02。R1冻结失败结果保留在 ../deployment-20261102-final/prepare-final.log 和其 frozen-release，不覆盖。

旧主underlay的 install/carcar_navigation/lib/libcarcar_monitored_stvl.so 在与服务同样的 main install/setup.bash + release local_setup.bash 环境中，缺少 libspatio_temporal_voxel_layer__rosidl_typesupport_cpp.so。当前正式unit也没有额外加载 /home/jetson/.local/share/carcar_stvl_humble/opt/ros/humble/lib。该文件在旧underlay和R1冻结中 SHA256相同，均为 ee9757b19d1593059c9eeb017f488a2f5fad67e67f20e141343675bf786867f4，属于历史未解决的独立深度插件依赖，而不是此次2D建图控制器增加的依赖。

R2审计**不补装/注入/掩盖任意新依赖**。仅当该唯一旧 ELF 源副本逐字节相同、在同一已source环境下二者的唯一缺失库集合严格相同，才把这项记录为 inherited_unchanged_missing；其他任何ELF、其他缺失项、额外缺失项或文件改变，都必须使prepare失败。

正式2D激光自动建图不加载这一STVL插件。但深度/体素导航用途依旧存在旧依赖缺口，不得将本轮2D软件闭环通过扩展成深度插件功能验收。后续如用D435/STVL，应专门验证该lib可加载和运行，不能依赖此基线例外直接上线。

本轮发布及回退不能更改原unit、原地图资产、原录包和现有深度模块运行环境。
