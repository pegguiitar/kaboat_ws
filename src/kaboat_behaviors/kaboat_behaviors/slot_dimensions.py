"""slot_dimensions — 도킹 슬롯·선체 치수의 검증과 파생값 (ROS 비의존).

슬롯 크기와 관련된 값은 `kaboat_bringup/config/dock_slot.yaml` 한 곳에서만
정한다. 노드는 그 값을 읽어 이 클래스로 만들고, 여기서 앞뒤가 안 맞는 치수를
시작 시점에 거부하며, 다른 곳에서 쓰는 파생값(후진 최소거리, 뒷벽 직선 창
반경)을 계산한다. 같은 치수를 launch·FSM·측정 모듈에 따로 적지 않도록 하는
것이 목적이다.

    wall_to_mouth   뒷벽(표식 벽) 면 → 슬롯 입구 면 수평 거리 [m]
    slot_width      슬롯 벽-벽 폭 [m]
    hull_width/len  선체 폭·전장 [m]
    berth_distance  접안 시 base_link(선체 중심) → 뒷벽 거리 [m]
    pre_entry_offset  입구 바깥 첫 목표점(W1)까지 거리 [m]
    exit_clearance  후진 완료 시 선미-입구 추가 여유 [m]
"""
from dataclasses import dataclass
import math

from .slot_axis import WallFitParams

MAX_WALL_TO_MOUTH = 8.0     # dock_target.plan_from_points 의 상한과 같다
MIN_BERTH_BOW_GAP = 0.1     # 접안 시 선수 끝~뒷벽 최소 여유 [m]


@dataclass(frozen=True)
class SlotDimensions:
    wall_to_mouth: float
    slot_width: float
    slot_width_tolerance: float
    hull_width: float
    hull_length: float
    berth_distance: float
    pre_entry_offset: float
    exit_clearance: float

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if not math.isfinite(value):
                raise ValueError(f'slot dimension {name} must be finite: {value}')
        if self.berth_distance <= 0 or self.pre_entry_offset <= 0 \
                or self.hull_length <= 0:
            raise ValueError('berth_distance, pre_entry_offset, hull_length '
                             'must be positive')
        if self.exit_clearance < 0 or self.slot_width_tolerance < 0:
            raise ValueError('exit_clearance and slot_width_tolerance '
                             'must be non-negative')
        if not 0 < self.hull_width < self.slot_width:
            raise ValueError(
                f'hull_width ({self.hull_width}) must be positive and smaller '
                f'than slot_width ({self.slot_width})')
        if not self.berth_distance + 0.2 < self.wall_to_mouth <= MAX_WALL_TO_MOUTH:
            raise ValueError(
                f'wall_to_mouth ({self.wall_to_mouth}) must be greater than '
                f'berth_distance + 0.2 ({self.berth_distance + 0.2}) and at '
                f'most {MAX_WALL_TO_MOUTH}')
        if self.berth_distance < self.hull_length / 2 + MIN_BERTH_BOW_GAP:
            raise ValueError(
                f'berth_distance ({self.berth_distance}) leaves the bow less '
                f'than {MIN_BERTH_BOW_GAP} m from the back wall for a '
                f'{self.hull_length} m hull')

    @property
    def reverse_min_distance(self) -> float:
        """접안 위치에서 선미가 입구 + 여유를 벗어날 때까지 후진해야 할 거리."""
        return max(0.5, self.wall_to_mouth + self.hull_length / 2
                   + self.exit_clearance - self.berth_distance)

    @property
    def wall_fit_radius(self) -> float:
        """표식 주변 뒷벽 직선 창 반경 — 슬롯 반폭에서 모서리 여유를 뺀 값."""
        return min(2.0, max(0.6, self.slot_width / 2 - 0.2))

    def wall_fit_params(self) -> WallFitParams:
        radius = self.wall_fit_radius
        return WallFitParams(window_radius=radius,
                             min_span=min(1.2, 1.6 * radius))

    def fsm_overrides(self) -> dict:
        """DockParams 필드 중 치수에서 정해지는 값."""
        return {
            'slot_width': self.slot_width,
            'slot_width_tolerance': self.slot_width_tolerance,
            'hull_width': self.hull_width,
            'hull_length': self.hull_length,
            'berth_distance': self.berth_distance,
            'pre_entry_offset': self.pre_entry_offset,
            'reverse_min_distance': self.reverse_min_distance,
        }
