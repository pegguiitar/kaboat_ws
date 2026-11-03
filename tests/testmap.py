"""시험 파일 분류표 — 단(level), 묶음(group), 기능(function).

루트 conftest.py 가 이 표로 marker 와 junit 속성을 붙인다.
새 시험 파일을 추가하면 여기에도 한 줄 추가한다. 빠지면 'unclassified' 로 표시된다.

level  : unit(L1 기능) / mission(L2 미션) / system(L3 전체 흐름) / tool(시험 도구 자체 검증)
         / excluded(보고서 범위 밖 — 수조·Gazebo 시험용 노드. 실행은 계속하지만 표에서 뺀다)
group  : 보고서 표의 행 묶음
function: 보고서에서 한 문단으로 설명하는 기능 이름
"""

GROUPS = {
    'mission_mgmt': '미션 관리',
    'common':       'behavior 공통',
    'gate':         'Gate (항로추종)',
    'station':      'Station (위치유지)',
    'dock':         'Dock (도킹)',
    'search':       'Search (탐색)',
    'avoid':        'Avoid (장애물 회피)',
    'perception':   '인지',
    'localization': '위치 추정',
    'control':      '제어',
    'tank_tools':   '수조 시험 도구',
    'test_infra':   '시험 기반',
}

# 시험 파일 이름(확장자 제외) → (level, group, function)
TEST_FILES = {
    # ── 미션 관리 ──
    'test_mission_manager':       ('unit', 'mission_mgmt', '미션 전환 (mission_manager)'),
    'test_mission_approach':      ('unit', 'mission_mgmt', '미션 시작 자세 접근 (mission_approach)'),
    'test_cmd_mux':               ('unit', 'mission_mgmt', '명령 선택·워치독 (cmd_mux)'),
    # ── behavior 공통 ──
    'test_behavior_buoys':        ('unit', 'common', '부표맵 상대 기하 (behavior_base)'),
    'test_obstacle_avoidance_utils': ('unit', 'common', '반발 조향 (apply_repulsion)'),
    'test_waypoint_controller':   ('unit', 'common', 'waypoint 제어'),
    'test_los_controller':        ('unit', 'common', 'LOS 경로 추종'),
    'test_motion_controller':     ('unit', 'common', '속도 벡터 제어'),
    # ── Gate ──
    'test_gate_corridor':         ('unit', 'gate', '게이트 통로 선택'),
    'test_gate_follower':         ('unit', 'gate', '게이트 노드 (gate_follower)'),
    'test_gate_follower_io':      ('unit', 'gate', '게이트 노드 (gate_follower)'),
    # ── Dock ──
    'test_dock_target':           ('unit', 'dock', '도킹 진입 경로 계산'),
    'test_docking_fsm':           ('unit', 'dock', '도킹 FSM'),
    'test_docking_ctrl':          ('unit', 'dock', '도킹 노드 (docking_ctrl)'),
    # ── Search ──
    'test_search_circler_los':    ('unit', 'search', '탐색 선회 노드 (search_circler)'),
    'test_search_circler':        ('unit', 'search', '탐색 선회 노드 (search_circler)'),
    # ── Station ──
    'test_station_keeper':        ('unit', 'station', '위치유지 노드 (station_keeper)'),
    # ── Avoid ──
    'test_dri':                   ('unit', 'avoid', 'DRI 위험도'),
    'test_bspline_planner':       ('unit', 'avoid', 'B-spline 경로 생성'),
    'test_avoid_fsm':             ('unit', 'avoid', '회피 FSM'),
    'test_obstacle_planner':      ('unit', 'avoid', '회피 노드 (obstacle_planner)'),
    # ── 인지 ──
    'test_buoy_tracker':          ('unit', 'perception', '부표 맵 (buoy_tracker)'),
    'test_buoy_detector':         ('unit', 'perception', '부표 색 검출 (buoy_detector)'),
    'test_occupancy_grid':        ('unit', 'perception', '점유 격자 (occupancy_grid)'),
    'test_dock_segmentation':     ('unit', 'perception', '도킹 표식 분할 뒤처리'),
    'test_depth_utils':           ('unit', 'perception', '깊이 영상 변환'),
    'test_depth_utils_bounds':    ('unit', 'perception', '깊이 영상 변환'),
    'test_dock_geometry':         ('unit', 'perception', '도킹 입구 추정'),
    'test_dock_geometry_bounds':  ('unit', 'perception', '도킹 입구 추정'),
    # ── 위치 추정 ──
    'test_indoor_lidar_odom':     ('unit', 'localization', '실내 라이다 odom'),
    'test_lidar_boat_tracker':    ('unit', 'localization', '외벽 라이다 선체 추적'),
    'test_lidar_sail_pose':       ('unit', 'localization', '┛ 판 자세 추정'),
    'test_lidar_panel_pose':      ('unit', 'localization', '판 자세 추정'),
    'test_lidar_marker_pose':     ('unit', 'localization', '마커 자세 추정'),
    'test_pose_velocity':         ('unit', 'localization', '자세 미분 속도'),
    'test_yaw_bias_ekf':          ('unit', 'localization', 'yaw bias EKF'),
    'test_imu_dead_reckoning':    ('unit', 'localization', 'IMU 추측 항법'),
    'test_onboard_scan_filter':   ('unit', 'localization', '선체 라이다 스캔 필터'),
    # ── 제어 ──
    'test_thruster_backend':      ('unit', 'control', '추진기 출력 변환'),
    'test_thruster_driver_teleop': ('unit', 'control', '추진기 드라이버·원격 조종'),
    'test_thruster_driver':       ('unit', 'control', '추진기 드라이버·원격 조종'),
    'test_health':                ('unit', 'control', '센서 상태 판정'),
    # ── 수조 시험 도구 — 보고서 범위 밖 (수조·Gazebo 시험은 현재 단계에서 제외) ──
    'test_straight_line_test':    ('excluded', 'tank_tools', '직진 시험 노드'),
    'test_bspline_track_test':    ('excluded', 'tank_tools', 'B-spline 추종 시험 노드'),
    'test_circle_drive_test':     ('excluded', 'tank_tools', '원 주행 시험 노드'),
    'test_los_path_test':         ('excluded', 'tank_tools', 'LOS 경로 시험 노드'),
    'test_waypoint_sequence_test': ('excluded', 'tank_tools', 'waypoint 순차 시험 노드'),
    'test_station_keeping_test':  ('excluded', 'tank_tools', '위치유지 시험 노드'),
    'test_real_obstacle_avoid_test': ('excluded', 'tank_tools', '실물 회피 시험 노드'),
    'test_manual_dri_monitor':    ('excluded', 'tank_tools', 'DRI 수동 모니터'),
    'test_tank_mission_guard':    ('excluded', 'tank_tools', '수조 미션 보호 장치'),
    'test_test_coordinates':      ('excluded', 'tank_tools', '수조 시험 좌표'),
    'test_trajectory_visualization': ('excluded', 'tank_tools', '궤적 시각화'),
    # ── L2 미션 시험 ──
    'test_mission_gate':          ('mission', 'gate', 'Gate 미션'),
    'test_mission_station':       ('mission', 'station', 'Station 미션'),
    'test_mission_dock':          ('mission', 'dock', 'Dock 미션'),
    'test_mission_search':        ('mission', 'search', 'Search 미션'),
    'test_mission_avoid':         ('mission', 'avoid', 'Avoid 미션'),
    # ── L3 전체 흐름 ──
    'test_mission_flow':          ('system', 'mission_mgmt', '전체 미션 흐름'),
    # ── 시험 기반 (선체 모델·가짜 센서 자체 검증) ──
    'test_sim_helpers':           ('tool', 'test_infra', '시험용 선체 모델·가짜 센서'),
}

LEVELS = ('unit', 'mission', 'system', 'tool', 'excluded')
