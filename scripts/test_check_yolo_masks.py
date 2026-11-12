"""check_yolo_masks.py 표시·저장 계산 (ROS 없이)."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import check_yolo_masks as cy  # noqa: E402


def _status(**rays):
    """dock_mark_detector 가 내는 detection_status JSON 모양 (필요한 칸만)."""
    return {'valid': False, 'reason': 'target_missing_or_invalid_depth', 'target_class': 'triangle',
            'inference_ms': 42.3, 'input_info': {'color': {'width': 640, 'height': 480}},
            'sight_rays': {name: {'pixel_uv': [u, 200.0], 'confidence': c, 'bearing_left_deg': b,
                                  'optical_ray': [0, 0, 1], 'depth_used': False}
                           for name, (u, c, b) in rays.items()}}


def test_frame_summary_reads_masks_without_depth():
    f = cy.frame_summary(_status(triangle=(318.0, 0.91, 1.2), square=(150.0, 0.55, 20.5)))
    assert set(f['masks']) == {'triangle', 'square'}
    assert f['masks']['triangle'] == {'confidence': 0.91, 'u': 318.0, 'v': 200.0, 'bearing_deg': 1.2}
    assert f['inference_ms'] == 42.3 and f['target'] == 'triangle' and f['size'] == (640, 480)
    empty = cy.frame_summary({'reason': 'disabled'})
    assert empty['masks'] == {} and empty['reason'] == 'disabled'


def test_rates_over_recent_frames():
    frames = [cy.frame_summary(_status(triangle=(320.0, 0.8, 0.0))) for _ in range(3)]
    frames.append(cy.frame_summary(_status()))
    stats = cy.class_stats(frames)
    assert stats['triangle'][0] == 0.75 and abs(stats['triangle'][1] - 0.8) < 1e-9
    assert stats['circle'] == (0.0, None)
    assert cy.class_stats([])['square'] == (0.0, None)


def test_line_shows_target_hits_misses_and_problems():
    frame = cy.frame_summary(_status(triangle=(318.0, 0.91, 1.2)))
    line = cy.format_line(frame, [frame], 9.8, '12:00:00')
    assert '9.8 Hz' in line and '42.3 ms' in line
    assert 'triangle* ✓0.91 (318,200) +1.2° 100%' in line     # * = 목표 도형
    assert 'circle ✗' in line and 'square ✗' in line
    broken = cy.frame_summary({'reason': 'model_unavailable', 'detail': "No module named 'torch'"})
    assert "YOLO 못 불러옴: No module named 'torch'" in cy.format_line(broken, [broken], 0.0, 't')
    assert '안 옴' in cy.format_line(None, [], 0.0, 't')


def _image(encoding, pixels):
    pixels = np.asarray(pixels, np.uint8)
    h, w, _ = pixels.shape
    step = w * 3 + 2                                          # 줄 끝 여백도 처리
    rows = np.zeros((h, step), np.uint8)
    rows[:, :w * 3] = pixels.reshape(h, w * 3)
    return SimpleNamespace(encoding=encoding, height=h, width=w, step=step, data=rows.tobytes())


def test_debug_image_to_bgr_and_save(tmp_path):
    bgr = [[[10, 20, 30], [40, 50, 60]]]
    np.testing.assert_array_equal(cy.image_to_bgr(_image('bgr8', bgr)), bgr)
    np.testing.assert_array_equal(cy.image_to_bgr(_image('rgb8', bgr)), [[[30, 20, 10], [60, 50, 40]]])
    path = cy.save_png(str(tmp_path / 'a.png'), cy.image_to_bgr(_image('bgr8', bgr)))
    assert Path(path).is_file() and Path(path).stat().st_size > 0


def test_summary():
    frames = [cy.frame_summary(_status(triangle=(320.0, c, 0.0))) for c in (0.9, 0.7)]
    text = '\n'.join(cy.summary_lines(frames, 3, '/tmp/x'))
    assert '프레임 2개' in text and 'triangle 검출률 100.0%' in text.replace('  ', ' ')
    assert '최저 0.70' in text and '3장 저장' in text
    assert cy.summary_lines([], 0, None) == ['받은 프레임 없음']
