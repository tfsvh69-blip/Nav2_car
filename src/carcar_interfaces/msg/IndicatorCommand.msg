# IndicatorCommand: Rosmaster 声光指示器控制命令
# 遵循规范：统一 RGB 提示、蜂鸣触发标志与持续毫秒数；携带时间戳、发送实例 ID 与单调递增序号。

builtin_interfaces/Time stamp
string sender_id
uint64 sequence
uint8 r
uint8 g
uint8 b
bool beep_trigger
uint16 beep_duration_ms

