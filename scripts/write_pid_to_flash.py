#!/usr/bin/env python3
"""一次性脚本：将运动 PID 写入控制板 Flash（永久保存）。

用法：
    1. 先停掉所有占用 /dev/myserial 的进程（驱动节点 Ctrl+C）。
    2. python3 write_pid_to_flash.py
    3. 完成后重启驱动。
"""

import sys
import time

# 把 Rosmaster_Lib 加入搜索路径
sys.path.insert(0, '/home/jetson/luhao/my_nav_carcar/src/rosmaster_vendor')

from Rosmaster_Lib import Rosmaster

SERIAL_PORT = '/dev/myserial'
CAR_TYPE = 1

# ===== 要写入的 PID 参数 =====
KP = 8.0
KI = 1.2
KD = 0.8
# ==============================

def main():
    print(f'目标 PID：Kp={KP}, Ki={KI}, Kd={KD}')
    print(f'串口：{SERIAL_PORT}')
    print()

    # 连接控制板
    print('正在连接控制板...')
    bot = Rosmaster(car_type=CAR_TYPE, com=SERIAL_PORT, debug=False)
    bot.create_receive_threading()
    time.sleep(0.5)

    # 先停车
    for _ in range(3):
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.05)

    # 读取当前 Flash PID
    print('读取当前 Flash PID...')
    before = bot.get_motion_pid()
    print(f'  写入前：Kp={before[0]}, Ki={before[1]}, Kd={before[2]}')
    print()

    # 写入 Flash（forever=True）
    print(f'正在写入 Flash：Kp={KP}, Ki={KI}, Kd={KD} ...')
    bot.set_pid_param(KP, KI, KD, forever=True)
    time.sleep(0.5)  # Flash 写入需要额外等待

    # 回读确认
    print('回读确认...')
    after = bot.get_motion_pid()
    print(f'  写入后：Kp={after[0]}, Ki={after[1]}, Kd={after[2]}')
    print()

    if (after[0] == KP and after[1] == KI and after[2] == KD):
        print('✓ Flash 写入成功！重启控制板后仍然生效。')
    else:
        print('✗ 警告：回读值与目标不一致，请检查！')
        print(f'  目标：Kp={KP}, Ki={KI}, Kd={KD}')
        print(f'  回读：Kp={after[0]}, Ki={after[1]}, Kd={after[2]}')

    # 断开
    bot.set_car_motion(0.0, 0.0, 0.0)
    del bot
    print('已断开串口。')


if __name__ == '__main__':
    main()
