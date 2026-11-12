#!/usr/bin/env python3
"""도킹 YOLO 마스크만 확인 — 뎁스·TF·라이다와 상관없이 도형 마스크가 잘 잡히는지.

젯슨에서 YOLO 를 먼저 켠다 (모터·미션 없이 D455 + 검출기, 항상 추론):
    bash scripts/run_dock_yolo.sh                  # run_camera 가 이미 켜져 있으면 start_camera:=false
그다음 다른 터미널에서:
    check_yolo                                     # = python3 scripts/check_yolo_masks.py
    check_yolo --save-every 0                      # 결과 영상 저장 안 함
    check_yolo --save-every 2 --out ~/yolo_imgs    # 2초마다 마스크 그림 PNG 저장

1초마다 한 줄: 처리 Hz, 추론 ms, 도형별 [✓ 신뢰도, 화면 픽셀(u,v), 방위각, 최근 프레임 검출률].
/dock/detection_status 의 sight_rays(마스크 8픽셀 이상이면 뎁스 없이도 나옴)를 본다 —
/detections/dock_marks 는 뎁스까지 통과한 것만이라 마스크 확인에는 안 맞는다.
/dock/debug_image(마스크·윤곽·상자·"도형 신뢰도"를 그린 320 px 영상)를 PNG 로 저장해
노트북에서 마스크 모양을 본다 (이 영상은 Zenoh 브리지로 안 넘어감).
Ctrl+C 로 끝내면 도형별 검출률·신뢰도·추론 시간 요약을 출력한다.
"""

import argparse
from collections import deque
import datetime
import json
import os
import time

CLASSES = ('triangle', 'circle', 'square')
WINDOW = 30            # 검출률을 보는 최근 프레임 수 (10 Hz 면 약 3 s)
STALE_S = 2.0          # 이보다 오래 상태가 없으면 '안 옴'
PROBLEM_REASONS = {    # 마스크를 못 보는 상태 → 그대로 보여 줌
    'disabled': '검출기 꺼짐 (/detector/enable false — run_dock_yolo.sh 는 항상 켬)',
    'model_unavailable': 'YOLO 못 불러옴',
    'image_or_inference_error': '추론 오류',
    'rgb_conversion_error': '컬러 영상 변환 오류',
    'unaligned_rgb_depth_or_info': '컬러·뎁스·camera_info 크기 다름',
    'invalid_intrinsics': 'camera_info 이상',
}


def frame_summary(data):
    """detection_status JSON → 한 프레임 요약 {masks: {도형: {...}}, inference_ms, reason, detail}."""
    rays = data.get('sight_rays') or {}
    masks = {}
    for name in CLASSES:
        ray = rays.get(name)
        if not ray:
            continue
        uv = ray.get('pixel_uv') or [None, None]
        masks[name] = {'confidence': float(ray.get('confidence', 0.0)),
                       'u': uv[0], 'v': uv[1],
                       'bearing_deg': ray.get('bearing_left_deg')}
    color = (data.get('input_info') or {}).get('color') or {}
    return {'masks': masks, 'inference_ms': data.get('inference_ms'),
            'reason': data.get('reason', ''), 'detail': data.get('detail', ''),
            'target': data.get('target_class', ''),
            'size': (color.get('width'), color.get('height'))}


def class_stats(frames):
    """최근 프레임 요약들 → {도형: (검출률 0~1, 평균 신뢰도 또는 None)}."""
    frames = list(frames)
    stats = {}
    for name in CLASSES:
        confs = [f['masks'][name]['confidence'] for f in frames if name in f['masks']]
        rate = len(confs) / len(frames) if frames else 0.0
        stats[name] = (rate, sum(confs) / len(confs) if confs else None)
    return stats


def format_line(latest, frames, hz, now_text):
    """1초마다 찍는 한 줄."""
    if latest is None:
        return f'[{now_text}] 검출기 상태 안 옴 — run_dock_yolo.sh·카메라(/camera/color/image_raw) 확인'
    stats = class_stats(frames)
    parts = []
    for name in CLASSES:
        rate, _ = stats[name]
        mark = '*' if name == latest['target'] else ''
        m = latest['masks'].get(name)
        if m:
            bearing = (f" {m['bearing_deg']:+.1f}°" if m['bearing_deg'] is not None else '')
            u = '?' if m['u'] is None else f"{m['u']:.0f}"
            v = '?' if m['v'] is None else f"{m['v']:.0f}"
            parts.append(f"{name}{mark} ✓{m['confidence']:.2f} ({u},{v}){bearing} {rate:4.0%}")
        else:
            parts.append(f'{name}{mark} ✗ {rate:4.0%}')
    infer = latest['inference_ms']
    infer_text = f'{infer:5.1f} ms' if isinstance(infer, (int, float)) else '  - ms'
    line = f"[{now_text}] {hz:4.1f} Hz | 추론 {infer_text} | " + ' | '.join(parts)
    problem = PROBLEM_REASONS.get(latest['reason'])
    if problem:
        line += f" | ⚠️ {problem}" + (f": {latest['detail']}" if latest['detail'] else '')
    return line


def image_to_bgr(msg):
    """sensor_msgs/Image (bgr8·rgb8) → H×W×3 BGR uint8 (cv_bridge 없이)."""
    import numpy as np
    encoding = msg.encoding.lower()
    if encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'unsupported debug image encoding {msg.encoding!r}')
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return image if encoding == 'bgr8' else image[:, :, ::-1]


def save_png(path, bgr):
    """cv2 가 있으면 PNG, 없으면 같은 이름의 .ppm (어느 쪽이든 이미지 뷰어로 열림)."""
    try:
        import cv2
        if cv2.imwrite(path, bgr):
            return path
    except ImportError:
        pass
    ppm = os.path.splitext(path)[0] + '.ppm'
    with open(ppm, 'wb') as f:
        f.write(f'P6 {bgr.shape[1]} {bgr.shape[0]} 255\n'.encode('ascii'))
        f.write(bgr[:, :, ::-1].tobytes())
    return ppm


def summary_lines(all_frames, saved, out_dir):
    if not all_frames:
        return ['받은 프레임 없음']
    lines = [f'프레임 {len(all_frames)}개']
    for name, (rate, conf) in class_stats(all_frames).items():
        confs = [f['masks'][name]['confidence'] for f in all_frames if name in f['masks']]
        conf_text = (f'신뢰도 평균 {conf:.2f}, 최저 {min(confs):.2f}' if confs else '신뢰도 -')
        lines.append(f'  {name:8s} 검출률 {rate:5.1%}  {conf_text}')
    infer = [f['inference_ms'] for f in all_frames if isinstance(f['inference_ms'], (int, float))]
    if infer:
        lines.append(f'  추론 평균 {sum(infer) / len(infer):.1f} ms, 최대 {max(infer):.1f} ms')
    if saved:
        lines.append(f'  결과 영상 {saved}장 저장 → {out_dir}')
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--save-every', type=float, default=5.0,
                        help='결과 영상(/dock/debug_image) PNG 저장 간격 [s], 0 = 저장 안 함 (기본 5)')
    parser.add_argument('--out', default='~/kaboat_yolo_checks',
                        help='저장 폴더 (아래에 날짜_시각 폴더를 만든다)')
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('check_yolo_masks')
    frames = deque(maxlen=WINDOW)
    all_frames = []
    stamps = deque(maxlen=WINDOW)
    state = {'latest': None, 'received': None, 'image': None, 'saved': 0, 'last_save': 0.0}
    out_dir = None
    if args.save_every > 0:
        out_dir = os.path.join(os.path.expanduser(args.out),
                               datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)

    def on_status(msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        summary = frame_summary(data)
        now = time.monotonic()
        state['latest'], state['received'] = summary, now
        frames.append(summary)
        all_frames.append(summary)
        stamps.append(now)

    def on_image(msg):
        state['image'] = msg

    node.create_subscription(String, '/dock/detection_status', on_status, 10)
    if out_dir:
        node.create_subscription(Image, '/dock/debug_image', on_image, qos_profile_sensor_data)
    print('YOLO 마스크 확인 — /dock/detection_status' +
          (f', 결과 영상 {args.save_every:g}초마다 → {out_dir}' if out_dir else '') +
          '  (✓ 신뢰도 (u,v) 방위각 최근검출률, * = 목표 도형, Ctrl+C 로 끝)', flush=True)
    try:
        while rclpy.ok():
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            fresh = state['received'] is not None and now - state['received'] < STALE_S
            hz = ((len(stamps) - 1) / (stamps[-1] - stamps[0])
                  if fresh and len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0)
            print(format_line(state['latest'] if fresh else None, frames, hz,
                              datetime.datetime.now().strftime('%H:%M:%S')), flush=True)
            if (out_dir and state['image'] is not None and fresh
                    and now - state['last_save'] >= args.save_every):
                found = '_'.join(sorted(state['latest']['masks'])) or 'none'
                name = datetime.datetime.now().strftime('%H%M%S') + f'_{found}.png'
                try:
                    save_png(os.path.join(out_dir, name), image_to_bgr(state['image']))
                    state['saved'] += 1
                except ValueError as exc:
                    print(f'⚠️ 결과 영상 저장 실패: {exc}', flush=True)
                state['last_save'] = now
    except KeyboardInterrupt:
        pass
    finally:
        print('\n'.join(['', '── 요약'] + summary_lines(all_frames, state['saved'], out_dir)),
              flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
