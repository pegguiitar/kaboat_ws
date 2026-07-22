# buoy_detector 가 /detections/buoys 로 발행하는 부표맵 (상시).
# header.frame_id = "odom" — buoys[].position 은 전역(odom) 좌표다.

std_msgs/Header header
Buoy[] buoys
