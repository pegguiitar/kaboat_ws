"""test_test_coordinates.py — test_coordinates 모듈 및 설정 유효성 단위 테스트."""

import os
import unittest
import yaml

from kaboat_hardware.test_coordinates import (
    STRAIGHT_LINE,
    BSPLINE_TRACK,
    BSPLINE_PRESETS,
    CIRCLE_DRIVE,
    STATION_KEEPING,
    get_bspline_control_points_xy,
)


class TestCoordinates(unittest.TestCase):
    """수조 테스트 좌표 설정 및 유효성 검증."""

    def test_straight_line_coords(self):
        """직선 주행 좌표가 수조(10m x 5m) 범위 내에 있는지 검증."""
        self.assertIn('start_x', STRAIGHT_LINE)
        self.assertIn('start_y', STRAIGHT_LINE)
        self.assertIn('goal_x', STRAIGHT_LINE)
        self.assertIn('goal_y', STRAIGHT_LINE)

        # 수조 안전 범위 (0 ~ 10m, 0 ~ 5m)
        self.assertTrue(0.0 <= STRAIGHT_LINE['start_x'] <= 10.0)
        self.assertTrue(0.0 <= STRAIGHT_LINE['start_y'] <= 5.0)
        self.assertTrue(0.0 <= STRAIGHT_LINE['goal_x'] <= 10.0)
        self.assertTrue(0.0 <= STRAIGHT_LINE['goal_y'] <= 5.0)

        # 출발점과 도착점이 달라야 함
        dist = ((STRAIGHT_LINE['goal_x'] - STRAIGHT_LINE['start_x'])**2 +
                (STRAIGHT_LINE['goal_y'] - STRAIGHT_LINE['start_y'])**2)**0.5
        self.assertGreater(dist, 1.0)

    def test_bspline_track_coords(self):
        """B-Spline 제어점 및 헬퍼 함수 유효성 검증."""
        cps = BSPLINE_TRACK.get('control_points', [])
        self.assertGreaterEqual(len(cps), 2)

        xs, ys = get_bspline_control_points_xy()
        self.assertEqual(len(xs), len(cps))
        self.assertEqual(len(ys), len(cps))

        for x, y in cps:
            self.assertTrue(0.0 <= x <= 10.0, f"X 좌표 {x}가 수조 경계를 벗어남")
            self.assertTrue(0.0 <= y <= 5.0, f"Y 좌표 {y}가 수조 경계를 벗어남")

    def test_bspline_presets(self):
        """B-Spline 사전 정의 프리셋들이 유효한 제어점을 가지는지 검증."""
        self.assertIn('s_curve', BSPLINE_PRESETS)
        self.assertIn('u_turn', BSPLINE_PRESETS)
        self.assertIn('diagonal', BSPLINE_PRESETS)
        self.assertIn('perimeter', BSPLINE_PRESETS)

        for name, pts in BSPLINE_PRESETS.items():
            self.assertGreaterEqual(len(pts), 2, f"프리셋 {name}의 점이 2개 미만")
            for x, y in pts:
                self.assertTrue(0.0 <= x <= 10.0, f"프리셋 {name} X좌표 {x} 오류")
                self.assertTrue(0.0 <= y <= 5.0, f"프리셋 {name} Y좌표 {y} 오류")

    def test_circle_drive_coords(self):
        """원형 주행 중심점 및 반경이 수조 내에 완전히 들어가는지 검증."""
        cx = CIRCLE_DRIVE['center_x']
        cy = CIRCLE_DRIVE['center_y']
        r = CIRCLE_DRIVE['radius']
        direction = CIRCLE_DRIVE['direction']

        self.assertIn(direction, ['ccw', 'cw'])
        self.assertGreater(r, 0.3)
        # 원 전체가 수조 (0~10, 0~5) 안에 포함되는지
        self.assertGreaterEqual(cx - r, 0.0)
        self.assertLessEqual(cx + r, 10.0)
        self.assertGreaterEqual(cy - r, 0.0)
        self.assertLessEqual(cy + r, 5.0)

    def test_station_keeping_coords(self):
        """웨이포인트 정점 유지 좌표 유효성 검증."""
        tx = STATION_KEEPING['target_x']
        ty = STATION_KEEPING['target_y']
        self.assertTrue(0.0 <= tx <= 10.0)
        self.assertTrue(0.0 <= ty <= 5.0)
        self.assertGreater(STATION_KEEPING.get('pos_deadband', 0.15), 0.0)
        self.assertGreater(STATION_KEEPING.get('yaw_deadband_deg', 8.0), 0.0)

    def test_yaml_file_integrity(self):
        """test_coordinates.yaml 파일이 유효한 YAML 구조를 갖추고 있는지 검증."""
        yaml_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'test_coordinates.yaml'
        )
        self.assertTrue(os.path.exists(yaml_path), f"YAML 파일 누락: {yaml_path}")
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        self.assertIn('straight_line_test', data)
        self.assertIn('bspline_track_test', data)
        self.assertIn('circle_drive_test', data)
        self.assertIn('station_keeping_test', data)


if __name__ == '__main__':
    unittest.main()
