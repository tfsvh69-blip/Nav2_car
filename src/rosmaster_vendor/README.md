# rosmaster_vendor

本包仅将项目资料中的 Yahboom `Rosmaster_Lib` V3.3.9 转为可由 colcon 安装的 vendor 包，未修改 `Rosmaster_Lib.py` 的串口协议实现。

- 原始压缩包：`ROS控制板相关资料/py_install_V3.3.9.zip`
- 原始压缩包 SHA-256：`1761c5873b6d1407afe5b4f4c5c4fb05787e07730448e786243071fe9e8b6ce7`
- 解压后驱动文件 SHA-256：`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`

上层代码只通过 `carcar_base` 使用此库，不应从算法或应用包直接导入它。原厂资料未附开源许可证，因此该 vendor 包标记为 `Proprietary`，项目根目录的 Apache-2.0 许可证不覆盖这里的原厂代码。
