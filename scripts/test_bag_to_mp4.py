"""bag_to_mp4.py: 기록한 bag 의 영상 토픽 → mp4."""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bag_to_mp4 as b2m  # noqa: E402

rosbag2_py = pytest.importorskip('rosbag2_py')


def _write_bag(path, topic, kind, frames, storage='sqlite3', compression=''):
    from rclpy.serialization import serialize_message
    if compression:
        writer = rosbag2_py.SequentialCompressionWriter(
            rosbag2_py.CompressionOptions(compression_format=compression,
                                          compression_mode=rosbag2_py.CompressionMode.MESSAGE))
    else:
        writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    try:                                     # Jazzy 는 id 가 필수, Humble 은 id 가 없음
        meta = rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr')
    except TypeError:
        meta = rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr')
    writer.create_topic(meta)
    for i, msg in enumerate(frames):
        writer.write(topic, serialize_message(msg), int(1e9 + i * 1e9 / 15))   # 15 Hz
    del writer


def _rgb(i, w=64, h=48):
    from sensor_msgs.msg import Image
    img = np.zeros((h, w, 3), np.uint8)
    img[:, :, 0] = 200                       # R 진하게 — mp4 에서 BGR 로 바뀌었는지 본다
    img[:, :8 + i, 1] = 255
    msg = Image(height=h, width=w, encoding='rgb8', step=w * 3)
    msg.data = img.tobytes()
    return msg


def _read_mp4(path):
    cap = cv2.VideoCapture(str(path))
    frames = []
    ok, frame = cap.read()
    while ok:
        frames.append(frame)
        ok, frame = cap.read()
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return frames, fps


def test_raw_rgb_bag_becomes_mp4_inside_the_bag_folder(tmp_path):
    bag = tmp_path / 'rgb_153828'
    _write_bag(bag, '/camera/color/image_raw', 'sensor_msgs/msg/Image',
               [_rgb(i) for i in range(10)])
    out, n, fps = b2m.convert(str(bag), log=lambda *_: None)
    assert out == str(bag / 'rgb_153828.mp4') and n == 10
    assert fps == pytest.approx(15.0, rel=1e-3)
    frames, video_fps = _read_mp4(out)
    assert len(frames) == 10 and frames[0].shape == (48, 64, 3)
    assert video_fps == pytest.approx(15.0, abs=0.1)
    b, _, r = frames[0][:, 40:].reshape(-1, 3).mean(axis=0)
    assert r > 150 and b < 60                # rgb8 → BGR 변환됨 (빨강이 빨강으로)


def test_compressed_topic_in_zstd_bag(tmp_path):
    from sensor_msgs.msg import CompressedImage
    bag = tmp_path / 'run_x'
    msgs = []
    for i in range(5):
        img = np.full((48, 64, 3), (0, 180, 0), np.uint8)
        msg = CompressedImage(format='jpeg')
        msg.data = cv2.imencode('.jpg', img)[1].tobytes()
        msgs.append(msg)
    _write_bag(bag, '/camera/color/image_raw/compressed', 'sensor_msgs/msg/CompressedImage',
               msgs, compression='zstd')
    out, n, _ = b2m.convert(str(bag), out=str(tmp_path / 'v.mp4'), fps=10, log=lambda *_: None)
    frames, fps = _read_mp4(out)
    assert n == 5 and len(frames) == 5 and fps == pytest.approx(10.0, abs=0.1)
    assert frames[0].reshape(-1, 3).mean(axis=0)[1] > 140


def test_avi_fallback_when_mp4_cannot_be_written(tmp_path):
    msgs = []
    writer, path = b2m.open_writer(str(tmp_path / 'a.mp4'), 15.0, (64, 48),
                                   log=msgs.append, fourccs=('XXXX', 'MJPG'))
    writer.write(np.zeros((48, 64, 3), np.uint8))
    writer.release()
    assert path == str(tmp_path / 'a.avi') and len(_read_mp4(path)[0]) == 1 and msgs


def test_pick_topic():
    topics = {'/odom': ('nav_msgs/msg/Odometry', 3),
              '/dock/debug_image': ('sensor_msgs/msg/Image', 2),
              '/camera/color/image_raw/compressed': ('sensor_msgs/msg/CompressedImage', 9)}
    assert b2m.pick_topic(topics) == '/camera/color/image_raw/compressed'
    assert b2m.pick_topic(topics, '/dock/debug_image') == '/dock/debug_image'
    with pytest.raises(SystemExit, match='영상이 아님'):
        b2m.pick_topic(topics, '/odom')
    with pytest.raises(SystemExit, match='영상 토픽이 없음'):
        b2m.pick_topic({'/odom': ('nav_msgs/msg/Odometry', 3)})
    assert b2m.fps_of(1, 0.0) == 15.0
