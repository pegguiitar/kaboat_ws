"""arena_20261114.json(오전) → arena_20261114pm.json: 2026-11-14 오후 run2_20261114_145941 탐색 부표 배치.

오후 탐색 부표는 오전보다 북쪽으로 2–4 m 옮겨져 있었다. 위치 = 그 bag /detections/buoys 트랙 중앙값
(id 91 검정 = 카메라 사진으로 짙은 캡슐 확인, 검출기 unknown / id 90 초록 / id 89 빨강). 탐색 구간
(370–448 s) 검출은 이 세 개뿐이었다. 미션 지점은 그 bag /mission_points/markers — 오전과 같다.
나머지(게이트·스테이션·도킹·회피)는 오전 그대로 — 탐색 시험(start_state:=search, scen/search_pm.json)용.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
A = json.load(open(f'{HERE}/arena_20261114.json'))
A['source'] = 'run2_20261114_145941 탐색 부표 (나머지는 arena_20261114.json 오전)'
PM = {'black': (171.25, -2.06), 'red': (175.97, 2.14), 'green': (168.30, 2.49)}
for c in A['circles']:
    if c['name'].startswith('search_'):
        c['x'], c['y'] = PM[c['name'][len('search_'):]]
A['search'] = {k: list(v) for k, v in PM.items()}
json.dump(A, open(f'{HERE}/arena_20261114pm.json', 'w'), indent=1)
print(A['search'])
