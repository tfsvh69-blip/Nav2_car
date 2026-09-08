#!/usr/bin/env python3
"""逐轮正转方向确认测试脚本。

测试顺序：
  1. 左上角（左前，M1 端口）: PWM = +50%, 运行 2.0 秒
  2. 右上角（右前，M3 端口）: PWM = +50%, 运行 2.0 秒
  3. 左下角（左后，M2 端口）: PWM = +50%, 运行 2.0 秒
  4. 右下角（右后，M4 端口）: PWM = +50%, 运行 2.0 秒

安全保证：
  - 启动前发送 3 次零输出，预留 3 秒倒计时准备。
  - 每个轮子运行严格受 2.0 秒看门狗控制，超时立即强制全停。
  - 轮次之间停顿 1.5 秒并保持零输出。
  - 支持 Ctrl+C 任意时刻急停并退出。
"""

import os
import sys
import time

# 导入底层驱动库
sys.path.insert(0, '/home/jetson/luhao/my_nav_carcar/src/rosmaster_vendor')
from Rosmaster_Lib import Rosmaster

SERIAL_PORT = '/dev/myserial'
PWM_SPEED = 50  # 50% 速度
RUN_DURATION = 2.0  # 每轮运行 2 秒
INTER_DELAY = 1.5  # 轮间停顿 1.5 秒
STARTUP_DELAY = 3  # 启动倒计时 3 秒

# 测试序列定义：(描述, 端口名称, (M1_pwm, M2_pwm, M3_pwm, M4_pwm))
TEST_SEQUENCE = [
    ('第 1/4 轮：左上角（左前轮）', 'M1 端口', (PWM_SPEED, 0, 0, 0)),
    ('第 2/4 轮：右上角（右前轮）', 'M3 端口', (0, 0, PWM_SPEED, 0)),
    ('第 3/4 轮：左下角（左后轮）', 'M2 端口', (0, PWM_SPEED, 0, 0)),
    ('第 4/4 轮：右下角（右后轮）', 'M4 端口', (0, 0, 0, PWM_SPEED)),
]


def stop_all(bot):
    """多次发送零输出确保彻底停车。"""
    for _ in range(3):
        try:
            bot.set_motor(0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.02)


def main():
    print('=' * 60)
    print('【电机正转方向确认测试】')
    print('测试顺序: 左上角(M1) -> 右上角(M3) -> 左下角(M2) -> 右下角(M4)')
    print(f'单轮参数: 正转 PWM = +{PWM_SPEED}%, 持续时间 = {RUN_DURATION} 秒')
    print('=' * 60)
    print('⚠️  安全警告：')
    print('  1. 请务必确认小车四轮已完全架空离开地面！')
    print('  2. 12 V 电机电源开关必须处于触手可及位置！')
    print('  3. 如遇异常动作，请立即断开 12 V 电源开关！')
    print('=' * 60)

    if not os.path.exists(SERIAL_PORT):
        print(f'❌ 错误：串口设备 {SERIAL_PORT} 不存在，请检查连接。')
        sys.exit(1)

    print(f'\n正在打开串口 {SERIAL_PORT} ...')
    bot = Rosmaster(com=SERIAL_PORT, debug=False)
    time.sleep(0.2)

    # 启动前连续发送 3 次零输出
    stop_all(bot)
    print('✓ 驱动连接成功，已发送启动零输出。')

    try:
        # 启动倒计时
        for sec in range(STARTUP_DELAY, 0, -1):
            print(f'⏳ 距离测试开始还有 {sec} 秒 (按 Ctrl+C 可随时中止)...')
            time.sleep(1.0)

        for idx, (desc, port, cmd) in enumerate(TEST_SEQUENCE, start=1):
            print('\n' + '-' * 50)
            print(f'▶️  开始测试 {desc} [{port}]')
            print(f'    指令: set_motor({cmd[0]}, {cmd[1]}, {cmd[2]}, {cmd[3]})')
            print(f'    转动中... 请观察该轮的机械转向（向前转还是向后转）')

            start_t = time.time()
            bot.set_motor(*cmd)

            # 严格限时运行，高频检测时间
            while (time.time() - start_t) < RUN_DURATION:
                time.sleep(0.05)

            # 超时立即停车
            stop_all(bot)
            print(f'⏹️  {desc} 动作结束，已停车。')

            if idx < len(TEST_SEQUENCE):
                print(f'⏸️  轮间安全停顿 {INTER_DELAY} 秒...')
                time.sleep(INTER_DELAY)

        print('\n' + '=' * 60)
        print('🎉 四轮测试序列全部完成！四轮已保持全停状态。')
        print('=' * 60)

    except KeyboardInterrupt:
        print('\n⚠️  捕获到键盘中断 (Ctrl+C)，立即紧急刹车！')
    finally:
        stop_all(bot)
        del bot
        print('已释放串口。测试脚本已安全退出。')


if __name__ == '__main__':
    main()
