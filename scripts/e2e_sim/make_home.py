"""판 폴더(HOME)에 미션 지점 파일·원점 파일을 쓴다 (arena.json 의 bag 기록 지점)."""
import json, sys, math
arena = json.load(open(sys.argv[1])); home = sys.argv[2]
sc = json.load(open(sys.argv[3])) if len(sys.argv) > 3 and sys.argv[3] else {}
o = arena['origin']
pts = dict(arena['points']); pts.update(sc.get('points', {}))
for name in sc.get('drop_points', []): pts.pop(name, None)
R = 6378137.0
out = {'version': 1, 'frame_id': 'odom', 'origin': {'lat': o['lat'], 'lon': o['lon'], 'alt': o['alt']}, 'points': {}}
for name, (x, y, yaw) in pts.items():
    out['points'][name] = {'x': x, 'y': y, 'yaw_deg': yaw, 'lat': o['lat'] + math.degrees(y / R),
                           'lon': o['lon'] + math.degrees(x / (R * math.cos(math.radians(o['lat'])))), 'recorded_at': 'bag 2026-11-13'}
json.dump(out, open(f'{home}/kaboat_mission_points.json', 'w'), ensure_ascii=False, indent=1)
open(f'{home}/kaboat_geo_origin.yaml', 'w').write(f"origin_lat: {o['lat']}\norigin_lon: {o['lon']}\norigin_alt: {o['alt']}\n")
