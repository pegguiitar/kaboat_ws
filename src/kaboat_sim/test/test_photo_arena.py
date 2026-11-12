"""Geometry checks runnable on the host without ROS or Gazebo."""
import copy
from collections import Counter, deque
import importlib.util
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent.parent
spec = importlib.util.spec_from_file_location(
    'generate_photo_arena', ROOT / 'scripts/generate_photo_arena.py')
arena = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arena)


def segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                   / (dx * dx + dy * dy)))
    return math.dist(point, (a[0] + t * dx, a[1] + t * dy))


class PhotoArenaTest(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(arena.CONFIG.read_text())
        self.model = ET.fromstring(arena.OUTPUT.read_text()).find('model')

    def test_committed_asset_matches_layout(self):
        self.assertEqual(arena.OUTPUT.read_text(), arena.generate(self.config))

    def test_obstacle_types_and_waterline(self):
        links = self.model.findall('link')
        names = [link.get('name') for link in links]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(sum(n.startswith('white_') for n in names), 22)
        self.assertEqual(sum(n.startswith('boom_') for n in names), 8)
        self.assertEqual(self.model.findtext('static'), 'true')
        for link in links:
            self.assertEqual(
                ET.canonicalize(ET.tostring(link.find('collision/geometry')), strip_text=True),
                ET.canonicalize(ET.tostring(link.find('visual/geometry')), strip_text=True))
        white_top = self.config['white_buoy']['center_z'] + self.config['white_buoy']['radius']
        for link in links:
            if not link.get('name').startswith('boom_'):
                continue
            z = float(link.findtext('pose').split()[2])
            height = float(link.findtext('collision/geometry/box/size').split()[2])
            self.assertLess(z - height / 2, 0)  # collision extends below water
            self.assertAlmostEqual(z + height / 2, 0.12)
            self.assertLess(z + height / 2, white_top / 2)

    def test_all_booms_attach_to_buoys_and_reach_their_ends(self):
        pairs = middle = boundary = 0
        width, depth = self.config['size']
        for boom in self.config['booms']:
            ends = [boom['start'], boom['end']]
            references = sum(isinstance(e, str) for e in ends)
            pairs += references == 2
            self.assertTrue(arena.attached_buoys(boom))
            middle += references == 0 and len(boom.get('attached_buoys', [])) == 2
            boundary += references == 1
            if references == 1:
                xy = next(e for e in ends if not isinstance(e, str))
                self.assertTrue(xy[0] in (0, width) or xy[1] in (0, depth))
            # Generated box reaches both endpoint centres with no end gap.
            link = self.model.find(f"link[@name='boom_{boom['name']}']")
            x, y, _, _, _, yaw = map(float, link.findtext('pose').split())
            length = float(link.findtext('collision/geometry/box/size').split()[0])
            for sign, end in zip((-1, 1), ends):
                actual = (x + sign * length / 2 * math.cos(yaw),
                          y + sign * length / 2 * math.sin(yaw))
                self.assertLess(math.dist(actual, arena.endpoint(self.config, end)), 1e-6)
        self.assertEqual((pairs, middle, boundary), (6, 1, 1))

    def test_three_large_buoys_per_column_and_mostly_vertical_booms(self):
        columns = Counter(n.split('_')[0] for n in self.config['buoys'] if n.startswith('c'))
        self.assertEqual(columns, {f'c{i}': 3 for i in range(1, 7)})
        self.assertGreaterEqual(2 * self.config['white_buoy']['radius'], 0.9)
        vertical = diagonal = 0
        for boom in self.config['booms']:
            a, b = [arena.endpoint(self.config, boom[key]) for key in ('start', 'end')]
            vertical += abs(a[0] - b[0]) < 1e-6
            diagonal += abs(a[0] - b[0]) > 1e-6 and abs(a[1] - b[1]) > 1e-6
        self.assertGreater(vertical, diagonal)
        self.assertIn(diagonal, (2, 3))

    def test_two_buoys_attach_inside_a_long_boom(self):
        boom = next(b for b in self.config['booms'] if b.get('attached_buoys'))
        a, b = [arena.endpoint(self.config, boom[key]) for key in ('start', 'end')]
        for name in boom['attached_buoys']:
            xy = self.config['buoys'][name]
            self.assertLess(segment_distance(xy, a, b), 1e-6)
            # Both buoy bodies lie inside the boom's ends, not at an endpoint.
            self.assertGreater(min(math.dist(xy, a), math.dist(xy, b)),
                               self.config['white_buoy']['radius'])

    def test_unrelated_buoys_do_not_intersect_booms(self):
        clearance = self.config['white_buoy']['radius'] + self.config['boom']['width'] / 2
        for boom in self.config['booms']:
            a, b = [arena.endpoint(self.config, boom[key]) for key in ('start', 'end')]
            for name, xy in self.config['buoys'].items():
                if name not in arena.attached_buoys(boom):
                    self.assertGreater(segment_distance(xy, a, b), clearance)

    def test_rectangle_rope_and_floats(self):
        width, depth = self.config['size']
        ropes = [n for n in self.model.findall('link') if n.get('name').startswith('boundary_rope_')]
        self.assertEqual(len(ropes), 6)
        self.assertAlmostEqual(sum(float(n.findtext('collision/geometry/cylinder/length'))
                                   for n in ropes), 2 * (width + depth) - 8.0)
        for link in self.model.findall('link'):
            if link.get('name').startswith('yellow_'):
                x, y, *_ = map(float, link.findtext('pose').split())
                self.assertTrue(x in (0, width) or y in (0, depth))
                if x in (0, width):
                    self.assertFalse(4.5 < y < 8.5, 'Gate blocked by a yellow float')

    def test_short_side_gates_are_open(self):
        segments = list(arena.boundary_segments(self.config))
        for gate in self.config['gates']:
            a, b = [arena.endpoint(self.config, gate[key]) for key in ('lower', 'upper')]
            middle = (a[0], (a[1] + b[1]) / 2)
            self.assertEqual(b[1] - a[1], 4.0)
            # No rope spans either entrance; white buoys occupy both endpoints.
            self.assertGreater(min(segment_distance(middle, c, d) for _, c, d in segments), 1.9)
            for key in ('lower', 'upper'):
                self.assertIsNotNone(self.model.find(f"link[@name='white_{gate[key]}']"))
        # Corner markers are larger than the white buoys and have an orange hue.
        self.assertGreater(self.config['corner']['radius'], self.config['white_buoy']['radius'])
        r, g, b, _ = self.config['corner']['color']
        self.assertGreater(r, g)
        self.assertGreater(g, b)

    def test_arena_is_in_existing_world(self):
        parent = ET.parse(PACKAGE / 'models/kaboat_2026_comprehensive_mission/model.sdf')
        include = parent.find(".//include[name='photo_avoidance']")
        self.assertEqual(include.findtext('uri'), 'model://kaboat_photo_avoidance')
        self.assertEqual(include.findtext('pose'), '5 11 0 0 0 0')
        world = ET.parse(PACKAGE / 'worlds/perception_task.sdf')
        self.assertIn('model://kaboat_2026_comprehensive_mission',
                      [n.text for n in world.findall('.//include/uri')])
        self.assertIsNotNone(parent.find(".//link[@name='dock_structure']"))
        self.assertIsNotNone(parent.find(".//include[name='gate_green_1']"))

    def test_boat_sized_route_enters_and_leaves_through_gates(self):
        # Conservative 1.3 m diameter footprint for the existing 1.1 m boat.
        # Geometric reachability only: this does not certify the live planner.
        radius, step = 0.65, 0.1
        width, depth = self.config['size']
        booms = [(arena.endpoint(self.config, n['start']), arena.endpoint(self.config, n['end']))
                 for n in self.config['booms']]
        ropes = [(a, b) for _, a, b in arena.boundary_segments(self.config)]

        def free(cell):
            x, y = cell[0] * step, cell[1] * step
            if not (-4 < x < width + 5 and radius < y < depth - radius):
                return False
            if any(math.dist((x, y), p) <= radius + self.config['white_buoy']['radius']
                   for p in self.config['buoys'].values()):
                return False
            if any(segment_distance((x, y), a, b) <= radius + self.config['boundary']['rope_radius']
                   for a, b in ropes):
                return False
            return all(segment_distance((x, y), a, b) > radius + self.config['boom']['width'] / 2
                       for a, b in booms)

        start, goal = (-30, 65), (310, 65)  # world (2, 77.5) -> (36, 77.5), 경기장 27 m (2026-11-12 실측)
        self.assertTrue(free(start))
        self.assertTrue(free(goal))
        pending, visited = deque([start]), {start}
        while pending:
            cell = pending.popleft()
            if cell == goal:
                return
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = cell[0] + dx, cell[1] + dy
                if nxt not in visited and free(nxt):
                    visited.add(nxt)
                    pending.append(nxt)
        self.fail('No boat-sized route through the photo arena')

    def test_rejects_zero_length_boom(self):
        broken = copy.deepcopy(self.config)
        broken['booms'][0]['end'] = broken['booms'][0]['start']
        with self.assertRaises(ValueError):
            arena.generate(broken)

    def test_rejects_unattached_or_disconnected_boom(self):
        broken = copy.deepcopy(self.config)
        broken['booms'] = [{'name': 'loose', 'start': [7, 1], 'end': [10, 1]}]
        with self.assertRaises(ValueError):
            arena.generate(broken)
        broken['booms'][0]['attached_buoys'] = ['c1_1']
        with self.assertRaises(ValueError):
            arena.generate(broken)


if __name__ == '__main__':
    unittest.main()
