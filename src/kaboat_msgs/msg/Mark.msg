# yolo_detector 가 인식한 표식/부표 1개.

string color        # "red" | "green" | "orange" | "blue" | "unknown"
string shape        # "circle" | "triangle" | "cross" | "unknown"  (YOLO 도입 전엔 unknown)
float32 confidence  # 0.0 ~ 1.0
float32 bearing     # 카메라 광축 기준 방위각 [rad], 좌측 +
float32 distance    # 추정 거리 [m], 미상이면 -1.0
