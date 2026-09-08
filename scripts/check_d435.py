#!/usr/bin/env python3
"""D435 维修后分阶段诊断；直接调用 SDK，不启动 ROS 或修改固件。"""

import argparse
import importlib.metadata
import json
import math
import statistics
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['enumerate', 'depth', 'color', 'both'],
                        default='enumerate')
    parser.add_argument('--seconds', type=int, choices=range(5, 301), default=30,
                        metavar='5..300')
    parser.add_argument('--serial', default='')
    parser.add_argument('--fps', type=int, choices=[6, 15, 30], default=15)
    args = parser.parse_args()
    report = {'mode': args.mode, 'result': '未完成', 'devices': []}
    pipeline = None
    started = False
    code = 1
    try:
        import pyrealsense2 as rs
        report['sdk'] = importlib.metadata.version('pyrealsense2')
        devices = list(rs.context().query_devices())
        for dev in devices:
            info = {}
            for key in ['name', 'serial_number', 'firmware_version',
                        'usb_type_descriptor']:
                field = getattr(rs.camera_info, key)
                info[key] = dev.get_info(field) if dev.supports(field) else '未知'
            info['profiles'] = sorted(set(
                str(p) for sensor in dev.query_sensors()
                for p in sensor.get_stream_profiles()))
            report['devices'].append(info)
        if not devices:
            raise RuntimeError('SDK 未发现设备：检查 USB、供电、权限和驱动；不能据此判定相机损坏')
        if args.mode == 'enumerate':
            report['result'] = '设备枚举成功；尚未验证成像'
            return_code = 0
        else:
            selected = [d for d in report['devices']
                        if not args.serial or d['serial_number'] == args.serial]
            if len(selected) != 1:
                raise RuntimeError('请选择唯一设备，使用 --serial 指定枚举出的序列号')
            if 'D435' not in selected[0]['name']:
                raise RuntimeError('选中设备不是 D435 系列，本脚本不自动尝试其他设备')
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(selected[0]['serial_number'])
            streams = (['depth', 'color'] if args.mode == 'both' else [args.mode])
            for name in streams:
                config.enable_stream(getattr(rs.stream, name), 640, 480,
                                     rs.format.z16 if name == 'depth' else rs.format.rgb8,
                                     args.fps)
            report['request'] = {'size': [640, 480], 'fps': args.fps,
                                 'seconds': args.seconds}
            pipeline.start(config)
            started = True
            stats = {s: {'unique_frames': 0, 'last': None, 'repeated': 0}
                     for s in streams}
            depth_ratios, distances, color_ranges = [], [], []
            timeouts = 0
            begin = time.monotonic()
            print('采样开始；请让相机面对0.5～1米处不反光的纸箱/墙面。', file=sys.stderr)
            while time.monotonic() - begin < args.seconds:
                try:
                    frames = pipeline.wait_for_frames(1500)
                except RuntimeError:
                    timeouts += 1
                    continue
                for name in streams:
                    frame = (frames.get_depth_frame() if name == 'depth'
                             else frames.get_color_frame())
                    if not frame:
                        continue
                    number = frame.get_frame_number()
                    if stats[name]['last'] == number:
                        stats[name]['repeated'] += 1
                        continue
                    stats[name]['last'] = number
                    stats[name]['unique_frames'] += 1
                    if name == 'depth':
                        values = [frame.get_distance(x, y)
                                  for y in range(120, 360, 24)
                                  for x in range(160, 480, 32)]
                        valid = [v for v in values if math.isfinite(v) and v > 0]
                        depth_ratios.append(len(valid) / len(values))
                        if valid:
                            distances.append(statistics.median(valid))
                    else:
                        values = bytes(frame.get_data())[::307]
                        color_ranges.append(max(values) - min(values))
            elapsed = time.monotonic() - begin
            for data in stats.values():
                data['received_fps'] = round(data['unique_frames'] / elapsed, 2)
            report.update(streams=stats, timeouts=timeouts)
            if depth_ratios:
                report['depth_center_valid_ratio_mean'] = statistics.mean(depth_ratios)
                report['depth_center_distance_median_m'] = (
                    statistics.median(distances) if distances else None)
            if color_ranges:
                report['color_sample_range_mean'] = statistics.mean(color_ranges)
            transport_ok = (timeouts == 0 and all(
                s['received_fps'] >= args.fps * 0.7 for s in stats.values()))
            report['result'] = ('出帧检查通过；图像质量与测距精度须人工复核'
                                if transport_ok else '出帧不稳定或缺失；需排查')
            if depth_ratios and statistics.mean(depth_ratios) < 0.5:
                report['scene_warning'] = '中央有效深度偏少：更换目标/距离再测，不能直接判为硬件损坏'
            return_code = 0 if transport_ok else 1
        code = return_code
    except KeyboardInterrupt:
        report['result'] = '用户中止，未完成测试'
        code = 130
    except Exception as error:
        report['result'] = '诊断未通过'
        report['error'] = str(error)
    finally:
        if started:
            try:
                pipeline.stop()
            except Exception as error:
                report['stop_error'] = str(error)
                code = 1
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return code


if __name__ == '__main__':
    sys.exit(main())
