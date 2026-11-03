"""dock_segmentation — YOLO 출력 뒤처리 (클래스당 1개, mask 크기, 클래스 이름 확인).

YOLO 추론 자체는 학습 모델이 필요해서 pytest 범위 밖이다. 여기서는 ultralytics 를
가짜 모듈로 바꿔, 모델이 낸 상자·mask 를 노드가 쓰는 형태로 정리하는 규칙만 본다.
"""
import sys
import types
from types import SimpleNamespace

import numpy as np
import pytest

from kaboat_perception.dock_segmentation import CLASS_NAMES, DockSegmenter


class _Scalar:
    def __init__(self, v):
        self.v = v

    def item(self):
        return self.v


class _Mask:
    def __init__(self, arr):
        self.arr = arr

    def cpu(self):
        return self

    def numpy(self):
        return self.arr


def _install_fake_yolo(monkeypatch, names, detections, mask_shape=(48, 64)):
    """detections: [(class_id, confidence, (row0, row1, col0, col1))] — mask 사각형."""
    boxes, masks = [], []
    for class_id, conf, (r0, r1, c0, c1) in detections:
        m = np.zeros(mask_shape, dtype=np.float32)
        m[r0:r1, c0:c1] = 1.0
        boxes.append(SimpleNamespace(cls=_Scalar(class_id), conf=_Scalar(conf)))
        masks.append(_Mask(m))
    result = SimpleNamespace(boxes=boxes,
                             masks=SimpleNamespace(data=masks) if detections else None)

    class FakeYolo:
        def __init__(self, weights):
            self.names = dict(enumerate(names))

        def predict(self, bgr, **kwargs):
            return [result]
    module = types.ModuleType('ultralytics')
    module.YOLO = FakeYolo
    monkeypatch.setitem(sys.modules, 'ultralytics', module)


def _image(h=48, w=64):
    return np.zeros((h, w, 3), dtype=np.uint8)


@pytest.mark.check('A')
def test_keeps_one_detection_per_class_sorted_by_confidence(monkeypatch):
    _install_fake_yolo(monkeypatch, CLASS_NAMES, [
        (0, 0.60, (0, 10, 0, 10)),
        (0, 0.90, (10, 20, 10, 20)),     # 같은 red_triangle — 더 높은 쪽만 남는다
        (2, 0.70, (20, 30, 20, 30)),
    ])
    out = DockSegmenter('fake.pt').predict(_image())
    assert [(d.class_name, d.confidence) for d in out] == [
        ('red_triangle', 0.90), ('blue_square', 0.70)]
    assert out[0].mask[15, 15] and not out[0].mask[5, 5]


@pytest.mark.check('A')
def test_one_marker_of_each_class_is_returned(monkeypatch):
    _install_fake_yolo(monkeypatch, CLASS_NAMES, [
        (2, 0.55, (0, 10, 0, 10)), (0, 0.80, (10, 20, 0, 10)), (1, 0.65, (20, 30, 0, 10))])
    out = DockSegmenter('fake.pt').predict(_image())
    assert [d.class_name for d in out] == ['red_triangle', 'green_circle', 'blue_square']
    assert all(d.mask.shape == (48, 64) and d.mask.any() for d in out)


@pytest.mark.check('B')
def test_mask_is_resized_to_image_size(monkeypatch):
    _install_fake_yolo(monkeypatch, CLASS_NAMES, [(1, 0.8, (0, 24, 0, 32))], mask_shape=(48, 64))
    out = DockSegmenter('fake.pt').predict(_image(96, 128))
    assert out[0].mask.shape == (96, 128)
    assert out[0].mask[40, 60] and not out[0].mask[60, 100]


@pytest.mark.check('B')
def test_unknown_class_id_is_skipped(monkeypatch):
    _install_fake_yolo(monkeypatch, CLASS_NAMES, [(5, 0.99, (0, 10, 0, 10)), (1, 0.5, (0, 10, 0, 10))])
    out = DockSegmenter('fake.pt').predict(_image())
    assert [d.class_name for d in out] == ['green_circle']


@pytest.mark.check('C')
def test_model_with_unexpected_classes_is_rejected(monkeypatch):
    _install_fake_yolo(monkeypatch, ('buoy', 'boat', 'dock'), [])
    with pytest.raises(ValueError):
        DockSegmenter('fake.pt')


@pytest.mark.check('C')
def test_no_masks_returns_empty(monkeypatch):
    _install_fake_yolo(monkeypatch, CLASS_NAMES, [])
    assert DockSegmenter('fake.pt').predict(_image()) == []
