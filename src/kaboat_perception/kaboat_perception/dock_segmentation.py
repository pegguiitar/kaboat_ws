"""YOLO26 segmentation adapter; one physical marker of each class per station."""

from dataclasses import dataclass

import cv2
import numpy as np


CLASS_NAMES = ("red_triangle", "green_circle", "blue_square")
CLASS_PARTS = {
    "red_triangle": ("red", "triangle"),
    "green_circle": ("green", "circle"),
    "blue_square": ("blue", "square"),
}


@dataclass(frozen=True)
class DockDetection:
    class_name: str
    confidence: float
    mask: np.ndarray


class DockSegmenter:
    def __init__(self, weights: str, confidence: float = 0.45,
                 image_size: int = 640, device: str = "cpu"):
        from ultralytics import YOLO

        self.model = YOLO(weights)
        self.confidence = confidence
        self.image_size = image_size
        self.device = device
        names = self.model.names
        found = tuple(names[index] for index in range(len(names)))
        if found != CLASS_NAMES:
            raise ValueError(f"Unexpected dock model classes: {found}")

    def predict(self, bgr: np.ndarray) -> list[DockDetection]:
        result = self.model.predict(
            bgr, conf=self.confidence, imgsz=self.image_size,
            device=self.device, retina_masks=True, verbose=False)[0]
        if result.masks is None:
            return []
        height, width = bgr.shape[:2]
        best = {}
        for index, box in enumerate(result.boxes):
            class_id = int(box.cls.item())
            if not 0 <= class_id < len(CLASS_NAMES):
                continue
            name = CLASS_NAMES[class_id]
            confidence = float(box.conf.item())
            if name in best and best[name].confidence >= confidence:
                continue
            # Keep the raster mask itself: polygon conversion may discard
            # islands separated by a post or other foreground occlusion.
            mask = result.masks.data[index].cpu().numpy() > 0.5
            if mask.shape != (height, width):
                mask = cv2.resize(mask.astype(np.uint8), (width, height),
                                  interpolation=cv2.INTER_NEAREST).astype(bool)
            best[name] = DockDetection(name, confidence, mask)
        return sorted(best.values(), key=lambda detection: detection.confidence,
                      reverse=True)
