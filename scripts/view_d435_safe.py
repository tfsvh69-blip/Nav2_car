#!/usr/bin/env python3
"""低负载显示 D435 彩色与深度画面，不修改固件或标定。"""

import argparse
import math
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='213222078719')
    parser.add_argument('--fps', type=int, choices=[6, 15], default=6)
    parser.add_argument('--max-depth', type=float, default=4.0)
    parser.add_argument('--rotate', type=int, choices=[0, 180], default=180,
                        help='仅旋转诊断显示，不修改相机数据或标定')
    args = parser.parse_args()
    if not math.isfinite(args.max_depth) or not 0.5 <= args.max_depth <= 10.0:
        parser.error('--max-depth 必须在 0.5～10.0 米之间')

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, args.fps)
    started = False
    window_name = 'D435 safe viewer - RGB | Depth (Q/ESC to quit)'

    try:
        profile = pipeline.start(config)
        started = True
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        usb_type = ('unknown' if not profile.get_device().supports(
            rs.camera_info.usb_type_descriptor) else profile.get_device().get_info(
                rs.camera_info.usb_type_descriptor))
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 480)

        last_number = {'depth': None, 'color': None}
        unique_pairs = 0
        begin = time.monotonic()
        last_report = begin
        print('窗口已启动：左侧 RGB，右侧伪彩深度；按 Q 或 Esc 退出。')
        while True:
            try:
                frames = pipeline.wait_for_frames(1500)
            except RuntimeError as error:
                print(f'等待图像超时：{error}', file=sys.stderr)
                continue
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            numbers = (depth_frame.get_frame_number(), color_frame.get_frame_number())
            if numbers == (last_number['depth'], last_number['color']):
                continue
            last_number['depth'], last_number['color'] = numbers
            unique_pairs += 1

            depth_raw = np.asanyarray(depth_frame.get_data())
            color_rgb = np.asanyarray(color_frame.get_data())
            if args.rotate == 180:
                depth_raw = cv2.rotate(depth_raw, cv2.ROTATE_180)
                color_rgb = cv2.rotate(color_rgb, cv2.ROTATE_180)
            color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
            depth_m = depth_raw.astype(np.float32) * depth_scale
            valid = np.isfinite(depth_m) & (depth_m > 0) & (depth_m <= args.max_depth)
            depth_u8 = np.clip(depth_m * (255.0 / args.max_depth), 0, 255).astype(np.uint8)
            depth_bgr = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
            depth_bgr[~valid] = 0

            height, width = depth_m.shape
            x0, x1 = width // 2 - 30, width // 2 + 30
            y0, y1 = height // 2 - 30, height // 2 + 30
            center = depth_m[y0:y1, x0:x1]
            center_valid = center[(center > 0) & np.isfinite(center)]
            center_distance = (float(np.median(center_valid))
                               if center_valid.size else float('nan'))
            valid_ratio = float(np.count_nonzero(valid)) / valid.size * 100.0
            elapsed = max(time.monotonic() - begin, 1e-6)
            measured_fps = unique_pairs / elapsed

            cv2.rectangle(depth_bgr, (x0, y0), (x1, y1), (255, 255, 255), 1)
            distance_text = ('center: no depth' if not math.isfinite(center_distance)
                             else f'center median: {center_distance:.3f} m')
            lines = [
                f'RGB 640x480 | Depth 0-{args.max_depth:g} m',
                distance_text,
                f'valid: {valid_ratio:.1f}%  fps: {measured_fps:.1f}  USB: {usb_type}',
            ]
            for index, line in enumerate(lines):
                y = 25 + index * 26
                cv2.putText(depth_bgr, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(depth_bgr, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (255, 255, 255), 1, cv2.LINE_AA)

            combined = np.hstack((color_bgr, depth_bgr))
            cv2.imshow(window_name, combined)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break

            now = time.monotonic()
            if now - last_report >= 5.0:
                print(f'frames={unique_pairs} fps={measured_fps:.2f} '
                      f'valid={valid_ratio:.1f}% center_m={center_distance:.3f}')
                last_report = now
        return 0
    except KeyboardInterrupt:
        print('收到 Ctrl+C，正在关闭相机流。')
        return 130
    except Exception as error:
        print(f'D435 窗口启动失败：{error}', file=sys.stderr)
        return 1
    finally:
        if started:
            try:
                pipeline.stop()
            except Exception as error:
                print(f'关闭相机流失败：{error}', file=sys.stderr)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    sys.exit(main())
