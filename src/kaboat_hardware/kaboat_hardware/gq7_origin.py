"""geo_origin 이 저장한 고정 원점 파일 읽기 (ROS 런타임 무의존).

원점 파일(~/kaboat_geo_origin.yaml)은 geo_origin(/geo/set_origin)이 쓰고, 야외 /odom 을
만드는 gq7_local_odom, 지점 기록(point_recorder), GPS 기록(gps_mark_logger),
mission_manager 가 읽는다.

예전에는 이 원점을 GQ7 드라이버 설정(filter_relative_position_*)에 넣어 /ekf/odometry_map 을
/odom 으로 썼지만, 젯슨 드라이버가 원점을 맞게 받고도 수백 km 떨어진 값을 내서(2026-11-11)
그 방식은 버렸다 — gq7_local_odom.py 참고.
"""

from kaboat_mission.mission_points import read_origin_file


def load_origin_file(path: str):
    """geo_origin 이 저장한 원점 파일 → (위도, 경도, 고도). 파일이 없으면 None.

    mission_manager 도 같은 파일을 읽으므로 구현은 kaboat_mission.mission_points 한 곳에 둔다.
    """
    return read_origin_file(path)
