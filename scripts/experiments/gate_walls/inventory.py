"""Read-only inventory of mission segments in a SQLite ROS 2 bag."""
import datetime as dt
import json
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

KST = dt.timezone(dt.timedelta(hours=9))
bag = Path('/bags/run_20261112_152344')
rows = []
for f in sorted(bag.glob('*.db3')):
    db = sqlite3.connect(f.as_uri() + '?mode=ro', uri=True)
    topics = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics')}
    ids = [i for i, (n, _) in topics.items() if n in ('/mission/state', '/mission/goal', '/mission/complete')]
    for ns, tid, raw in db.execute('select timestamp,topic_id,data from messages where topic_id in (' + ','.join('?' for _ in ids) + ') order by timestamp', ids):
        name, typ = topics[tid]
        msg = deserialize_message(raw, get_message(typ))
        value = msg.data if hasattr(msg, 'data') else [msg.pose.position.x, msg.pose.position.y]
        item = [dt.datetime.fromtimestamp(ns/1e9, KST).isoformat(), name, value]
        if not rows or item[1:] != rows[-1][1:]:
            rows.append(item)
print(json.dumps(rows, ensure_ascii=False, indent=2))
