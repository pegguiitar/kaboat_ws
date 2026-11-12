#!/usr/bin/env python3
"""ROS bag 의 카메라 영상 토픽 → mp4 (동영상 플레이어로 보기).

  python3 scripts/bag_to_mp4.py <bag 폴더> [--topic /camera/color/image_raw] [-o 출력.mp4] [--fps N]
  단축어(젯슨): bag_to_mp4 rgb_153828

ros2 bag record 로 찍은 폴더(.db3 / .mcap)와 rec_bag 폴더(압축 토픽, zstd 압축)를 그대로 받는다.
토픽을 안 주면 /camera/color/image_raw → 그 /compressed → 첫 영상 토픽 순으로 고른다.
fps 를 안 주면 기록 개수 ÷ 기록 시간(보통 15). 출력 기본값은 bag 폴더 안 <폴더 이름>.mp4.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

IMAGE_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
PREFERRED = ('/camera/color/image_raw', '/camera/color/image_raw/compressed')


def bag_info(bag):
    """metadata.yaml → (storage_id, compression_format, {토픽: (타입, 개수)}, 기록 시간 s)."""
    with open(os.path.join(bag, 'metadata.yaml'), encoding='utf-8') as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    topics = {t['topic_metadata']['name']: (t['topic_metadata']['type'], int(t['message_count']))
              for t in info.get('topics_with_message_count') or []}
    duration = float((info.get('duration') or {}).get('nanoseconds', 0)) * 1e-9
    return info.get('storage_identifier', ''), info.get('compression_format', ''), topics, duration


def pick_topic(topics, wanted=None):
    """영상 토픽 고르기. wanted 가 있으면 그것만(영상 타입이어야 함)."""
    images = [name for name, (kind, _) in topics.items() if kind in IMAGE_TYPES]
    if wanted:
        if wanted not in images:
            raise SystemExit(f'토픽 {wanted} 가 없거나 영상이 아님. 영상 토픽: {images or "없음"}')
        return wanted
    for name in PREFERRED:
        if name in images:
            return name
    if not images:
        raise SystemExit(f'영상 토픽이 없음. 기록된 토픽: {sorted(topics)}')
    return sorted(images)[0]


def fps_of(count, duration, default=15.0):
    if count >= 2 and duration > 0:
        return (count - 1) / duration
    return default


def image_to_bgr(msg):
    """sensor_msgs Image / CompressedImage → BGR uint8 (h, w, 3)."""
    if hasattr(msg, 'format'):                                  # CompressedImage
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f'압축 영상 해제 실패 (format={msg.format})')
        return frame
    enc = msg.encoding.lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1, '8uc3': 3}
    if enc not in channels:
        raise ValueError(f'영상 인코딩 {msg.encoding} 은 지원 안 함 ({sorted(channels)})')
    n = channels[enc]
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * n].reshape(msg.height, msg.width, n)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if n == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


def open_reader(bag, storage_id, compression):
    import rosbag2_py
    reader = (rosbag2_py.SequentialCompressionReader() if compression
              else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def open_writer(out, fps, size, log=print, fourccs=('mp4v', 'MJPG')):
    """mp4(mp4v) 를 연다. OpenCV 가 ffmpeg 없이 빌드돼 mp4 를 못 쓰면 같은 이름 .avi(MJPG)로."""
    for fourcc in fourccs:
        path = out if fourcc == 'mp4v' else os.path.splitext(out)[0] + '.avi'
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), fps, size)
        if writer.isOpened():
            if path != out:
                log(f'  이 OpenCV 는 mp4 를 못 씀 → {path} (MJPG)')
            return writer, path
        writer.release()
    raise SystemExit(f'영상 파일을 만들 수 없음: {out}')


def convert(bag, out=None, topic=None, fps=None, log=print):
    """bag 의 영상 토픽을 mp4 로. 반환 (출력 경로, 장 수, fps)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = os.path.abspath(os.path.expanduser(bag.rstrip('/')))
    storage_id, compression, topics, duration = bag_info(bag)
    topic = pick_topic(topics, topic)
    kind, count = topics[topic]
    fps = float(fps) if fps else fps_of(count, duration)
    out = out or os.path.join(bag, os.path.basename(bag) + '.mp4')
    log(f'{bag}\n  토픽 {topic} ({kind}), {count}장, {fps:.1f} fps → {out}')

    reader = open_reader(bag, storage_id, compression)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(kind)
    writer, size, frames = None, None, 0
    try:
        while reader.has_next():
            _, data, _ = reader.read_next()
            frame = image_to_bgr(deserialize_message(data, msg_type))
            if writer is None:
                size = (frame.shape[1], frame.shape[0])
                writer, out = open_writer(out, fps, size, log)
            elif (frame.shape[1], frame.shape[0]) != size:
                frame = cv2.resize(frame, size)
            writer.write(frame)
            frames += 1
            if frames % 300 == 0:
                log(f'  {frames}/{count}장')
    finally:
        if writer is not None:
            writer.release()
    if frames == 0:
        raise SystemExit(f'{topic} 에 영상이 한 장도 없음')
    log(f'✓ {frames}장, {frames / fps:.1f} 초 → {out}')
    return out, frames, fps


def main(argv=None):
    ap = argparse.ArgumentParser(description='ROS bag 영상 토픽 → mp4')
    ap.add_argument('bag', help='bag 폴더 (metadata.yaml 이 있는 곳)')
    ap.add_argument('--topic', help='영상 토픽 (기본: /camera/color/image_raw 또는 그 /compressed)')
    ap.add_argument('-o', '--out', help='출력 mp4 (기본: bag 폴더 안 <폴더 이름>.mp4)')
    ap.add_argument('--fps', type=float, help='재생 속도 (기본: 기록 개수 ÷ 기록 시간)')
    args = ap.parse_args(argv)
    if not os.path.exists(os.path.join(os.path.expanduser(args.bag), 'metadata.yaml')):
        sys.exit(f'{args.bag} 에 metadata.yaml 이 없음 — bag 폴더를 줄 것 (.db3 파일 말고)')
    convert(args.bag, args.out, args.topic, args.fps)


if __name__ == '__main__':
    main()
