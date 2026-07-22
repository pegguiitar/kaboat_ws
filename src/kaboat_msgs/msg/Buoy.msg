# buoy_detector 가 추적하는 부표 1개 — 월드프레임의 지속 랜드마크.
# 점유지도(cell)와 달리 색·정체성·전역좌표를 갖는 "점 랜드마크"다.

uint32 id                     # 안정 트랙 id (같은 부표는 프레임이 바뀌어도 같은 id)
string color                  # "red" | "green" | "orange" | "unknown"
geometry_msgs/Point position  # odom 프레임 좌표 [m], z 는 미사용(0)
float32 confidence            # 0.0 ~ 1.0
