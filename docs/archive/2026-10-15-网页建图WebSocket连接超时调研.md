# 网页建图 WebSocket 连接超时调研

日期：2026-10-15，范围：R11 的 Boost Beast 连接空闲回收。本文只核对官方固定源码与本机已安装头文件，不修改代码、系统或硬件。当前结论与实机证据见[测试记录](../测试记录.md)，完整有效命令见[操作手册](../操作手册.md)；RTC 与静止就绪的前置取舍见[WebRTC 控制通道调研](2026-10-15-网页建图WebRTC控制通道调研.md)。

## 1. 当前现象与可得出的结论

主代理本轮观察到真实客户端 `connection_generation` 已到 21，若干 `controller_disconnected` 发生在最后客户端诊断事件约 2 秒之后；此前 HTTP 曾超时，控制间隔仍有 7～115 毫秒的读数。本项目源码当时设置 WebSocket `idle_timeout=2s`、`keep_alive_pings=true`，且 RTC 仍依赖该 WebSocket 持有唯一操作者。

这些现象与“WebSocket 接收暂时停顿，被连接空闲回收连带关闭 RTC”的路径相符，但尚未保存关闭时的首个错误码与来源。客户端诊断事件不是全部 WebSocket 收包记录，HTTP 请求也是另一条连接；不能据此判定现场唯一根因。`connection_generation` 的增长只证明连接代次增加，不解释为何关闭。

## 2. 固定版本与官方机制

参考版本为 Boost Beast `boost-1.74.0`，固定提交 `201105e66ab738bd8e027cff8c11ddd7cd5c7587`，采用 Boost Software License 1.0。已只读核对本机 `/usr/include/boost/version.hpp` 为 `BOOST_VERSION=107400`；下列四份已安装头文件与该官方提交对应文件逐字节一致。[固定官方源码树](https://github.com/boostorg/beast/tree/201105e66ab738bd8e027cff8c11ddd7cd5c7587)、[官方许可证](https://github.com/boostorg/beast/blob/boost-1.74.0/LICENSE_1_0.txt)。

| 官方固定源码 | 与本题直接相关的职责 |
|---|---|
| [stream_base.hpp](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/stream_base.hpp) | 定义握手/关闭超时、连接空闲超时及自动 ping；功能用于异步操作，接收端须持续保有读操作 |
| [impl/stream_impl.hpp](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/stream_impl.hpp) | 空闲计时分段、计数复位、底层 socket 关闭以及超时错误交付 |
| [impl/read.hpp](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/read.hpp) | 接收底层 WebSocket 数据后复位空闲计数，内部处理 ping/pong 控制帧 |
| [impl/ping.hpp](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/ping.hpp) | 自动空闲 ping 的发送操作；需要取得 WebSocket 写锁 |

`idle_timeout` 判断 WebSocket 接收活动；它不读取 HTTP、RTC DataChannel 或 ROS 传感器是否仍正常。该版本启用自动 ping 后，`update_timer()` 先等待配置时长的一半；首次到期安排 ping、增加空闲计数，再等待另一半；仍未接收数据时执行 `time_out()`。因此，在持续有异步读、没有任何接收且调度及时的典型条件下，2 秒配置对应约 1 秒发 ping、约 2 秒回收，而不是等 2 秒发 ping 后再等 2 秒。[固定计时实现](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/stream_impl.hpp)。

接收数据会清除空闲计数；ping/pong 可在读操作内部处理，不必产生应用文本消息的完成回调。因此“最后一条客户端诊断的时间”不能直接代替“最后收到 WebSocket 数据的时间”。自动 ping 也需写锁，可能等待已有写操作，不能绕过 TCP 的阻塞和重传。[接收与控制帧处理](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/read.hpp)、[自动 ping 写锁](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/ping.hpp)。

`time_out()` 标记超时、改变连接状态并直接关闭底层 socket；首个检查到该状态的操作取得 `beast::error::timeout`，其他随后取消的操作可能取得 `operation_aborted`。若应用只把所有错误统一转成“连接断开”，就会丢失这一区别。连接计时使用 steady clock，事件循环调度和实际接收可能改变观察到的时间，不能把配置值当作每次断开的精确秒表结果。[固定关闭与错误转换实现](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/impl/stream_impl.hpp)。

官方服务器建议值为握手/关闭 30 秒、连接空闲 300 秒；本项目当时显式覆盖为握手/关闭 3 秒、连接空闲 2 秒。这只是通用库默认与项目配置的差异，不代表通用默认值适合本车，也不能把该 300 秒与本车 300 毫秒运动期限混淆。[固定建议值定义](https://github.com/boostorg/beast/blob/201105e66ab738bd8e027cff8c11ddd7cd5c7587/include/boost/beast/websocket/stream_base.hpp)。

## 3. 连接回收与运动截止的职责区别

| 项目机制 | 输入与判断 | 失败后的职责 |
|---|---|---|
| WebSocket 连接空闲回收 | WebSocket 自身的接收与协议 ping/pong | 回收无响应连接，释放连接资源和操作者占用 |
| Safety 的 300 毫秒运动截止 | 车端签发的挑战、当前会话/序号与当次运动有效期 | 过期运动不能继续驱动，按独立门控周期输出零速 |

两条传输路径可有不同表现：TCP 中旧数据丢失需要恢复时，RTC 的 UDP 数据仍可能正常到达。健康 RTC 不会自动重置 Beast 的 WebSocket 空闲计数；本项目的 WebSocket `close()` 又会关闭 RTC、撤销操作者，因此代码上存在“UDP 控制仍新鲜，却被 WebSocket 空闲回收连带锁停”的可能。这是源码路径推断，实际现场必须用关闭错误与同时段收包诊断证实。

反过来，WebSocket 还能回复 ping/pong，也不能证明 RTC 运动请求仍有效。运动路径断流仍须由 Safety 在 300 毫秒有效期后停车；不得用 WebSocket 存活延长旧运动请求。静止就绪按 R10 规则保持零输出，不以操纵心跳维持运动许可。

## 4. 10 秒连接回收、300 毫秒运动截止的候选取舍

本项目候选为 WebSocket `idle_timeout=10s`、继续启用自动 ping，Safety 运动截止保持 300 毫秒，握手/关闭期限独立维护。按该固定实现，典型无接收条件下约 5 秒安排 ping、约 10 秒回收；实际错误到达时间仍受接收和事件循环调度影响。

- 收益：为短暂 TCP 停顿留出恢复空间，减少连接回收导致的重建与 RTC 连带锁停；不为旧运动增加有效期。
- 代价：完全沉默的 WebSocket、其资源及操作者占用可能保留更久；若 RTC 仍持续提供有效控制，则在连接回收前仍按现有门控条件处理它。
- 边界：主动断开、页面隐藏、RTC 故障或传感器故障继续走现有锁停规则；恢复或重连不能自动恢复旧运动。10 秒不是允许断流后继续运动 10 秒，也不是网络修复。

## 5. 必须补的关闭诊断与验证

当前应用读/写完成回调遇错误后直接进入同一关闭函数，尚缺首个错误原因。应在第一次关闭前关联连接代次、阶段和原因，记录错误 `category/value/message`、可用的 WebSocket close code/reason、最近应用消息及 ping/pong 时间、待发/正在发送情况和同期 RTC 状态。首个超时不能被后续 `operation_aborted` 覆盖；日志不记录会话凭据原文。

先在替身底盘验证：TCP 暂停超过 2 秒但少于 10 秒且 RTC 保持新鲜时，候选连接能否继续；WebSocket 持续沉默时能否按配置回收；RTC 运动断流时即使 WebSocket 健康，300 毫秒有效期仍失效并输出零速；主动断开、隐藏、传感器故障及重连仍符合锁停规则。再对真实浏览器同时记录关闭原因、WS/RTC 间隔和代次增长，判断减少的是哪类断开。

该归档只证明 Boost 固定实现和候选取舍的依据，未执行网络故障注入或机器人测试，未确认现场唯一根因，也未证明实际 Wi-Fi 已修复。
