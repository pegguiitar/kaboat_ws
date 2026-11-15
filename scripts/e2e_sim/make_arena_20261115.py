"""arena_20261114pm.json → arena_20261115.json: 2026-11-15 오전 run3_20261115_095518 탐색 부표 배치.

위치 = 그 bag 탐색 구간 /detections/buoys 트랙 마지막 값 (id 75 검정 unknown 577회 / id 74 초록 632회 /
id 73 빨강 545회 — 탐색 중 영역 안 검출은 이 셋뿐). 11-14 오후보다 초록이 북서쪽으로 약 3 m 옮겨졌다.
미션 지점(search entry/end)은 같은 bag 기록 지점과 같아 그대로. 나머지는 11-14 오후 그대로 — 탐색 시험용.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
A = json.load(open(f'{HERE}/arena_20261114pm.json'))
A['source'] = 'run3_20261115_095518 탐색 부표 (나머지는 arena_20261114pm.json)'
P = {'black': (169.91, -1.69), 'red': (174.97, 2.63), 'green': (166.58, 4.86)}
for c in A['circles']:
    if c['name'].startswith('search_'):
        c['x'], c['y'] = P[c['name'][len('search_'):]]
A['search'] = {k: list(v) for k, v in P.items()}
json.dump(A, open(f'{HERE}/arena_20261115.json', 'w'), indent=1)
print(A['search'])
