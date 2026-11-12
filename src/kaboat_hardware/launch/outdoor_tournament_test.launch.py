"""토너먼트 장애물 회피 시험: rec 기록 지점 tournament_entry 에서 출발해 tournament_end 까지.

장애물 회피를 종합 자율 미션과 별개로 따로 하는 경기장(토너먼트)용이다. 노드·출발 조건·
인자는 야외 회피 시험(outdoor_avoid_test.launch.py, real_obstacle_avoid_test)과 같고 기본
지점 이름만 다르다. 종합 미션(mission_manager)은 tournament_* 지점을 읽지 않는다.
센서·모터·지도는 따로: run_gq7_rtk, run_lidar, run_motor, run_map.

  rec tournament_entry                    # 0) 노트북: 경기장 입구에 진행 방향으로 세우고 기록
  rec tournament_end                      #    도착(목표) 지점에서 기록
  goto tournament_entry                   # 1) 시작점에 기록 선수각으로 세움 (도착 후 Ctrl+C)
  test_tournament drive_enabled:=false    # 2) 미리보기: 경로만 (RViz /real_avoid_test/path)
  test_tournament                         # 3) 출발 조건이 맞으면 바로 출발

출발 조건: 시작점 start_tolerance(3 m) 안, 선수가 목표 방향에서 45° 이내. 목표 2 m 안이면 끝.
나머지 인자(drive_enabled, goal_tolerance, max_cross_track_error …)는 test_avoid_outdoor 와 같다.
"""

import importlib.util
import os

# 같은 launch 폴더(설치된 share 또는 src)의 회피 시험 launch 를 그대로 다시 쓴다
_AVOID_LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'outdoor_avoid_test.launch.py')

POINTS = {'start_point': 'tournament_entry', 'goal_point': 'tournament_end'}


def _load_avoid_launch():
    spec = importlib.util.spec_from_file_location('outdoor_avoid_test_launch', _AVOID_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_avoid = _load_avoid_launch()
ARGS = [(name, POINTS.get(name, default), text) for name, default, text in _avoid.ARGS]
avoid_parameters = _avoid.avoid_parameters


def generate_launch_description():
    return _avoid.generate_launch_description(ARGS)
