# LiDAR 클러스터링으로 확정된 장애물 1개.
# position 좌표계는 ObstacleArray.header.frame_id 를 따른다
# (odom TF 가용 시 odom, 아니면 lidar_link).

geometry_msgs/Point position   # 장애물 중심 [m]
float32 radius                 # 근사 반지름 [m]
