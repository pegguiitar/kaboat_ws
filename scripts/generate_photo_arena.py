#!/usr/bin/env python3
"""Generate a self-contained, anchored SDF arena using metres and RGBA colors.

No ROS/Gazebo installation is needed. The committed SDF is installed by CMake;
run this again after editing config/photo_arena.json. --check detects stale SDF.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/kaboat_sim'
CONFIG = PACKAGE / 'config/photo_arena.json'
OUTPUT = PACKAGE / 'models/kaboat_photo_avoidance/model.sdf'


def values(numbers):
    return ' '.join(f'{v:.8g}' for v in numbers)


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def endpoint(config, value):
    return tuple(config['buoys'][value] if isinstance(value, str) else value)


def attached_buoys(boom):
    """Buoys can attach at the segment ends or partway along its length."""
    return {boom[key] for key in ('start', 'end') if isinstance(boom[key], str)} | set(
        boom.get('attached_buoys', []))


def segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                   / (dx * dx + dy * dy)))
    return math.dist(point, (a[0] + t * dx, a[1] + t * dy))


def sphere(radius):
    geometry = ET.Element('geometry')
    element(element(geometry, 'sphere'), 'radius', radius)
    return geometry


def cylinder(radius, length):
    geometry = ET.Element('geometry')
    shape = element(geometry, 'cylinder')
    element(shape, 'radius', radius)
    element(shape, 'length', length)
    return geometry


def box(size):
    geometry = ET.Element('geometry')
    element(element(geometry, 'box'), 'size', values(size))
    return geometry


def boundary_segments(config):
    """Rectangle edges excluding the open water between each gate's white buoys."""
    width, depth = config['size']
    edges = [('south', (0, 0), (width, 0)), ('east', (width, 0), (width, depth)),
             ('north', (width, depth), (0, depth)), ('west', (0, depth), (0, 0))]
    for side, a, b in edges:
        holes = []
        for gate in config['gates']:
            if gate['side'] == side:
                ts = [(endpoint(config, gate[key])[1] - a[1]) / (b[1] - a[1])
                      for key in ('lower', 'upper')]
                holes.append(sorted(ts))
        cursor = 0.0
        for index, (lo, hi) in enumerate(sorted(holes) + [(1.0, 1.0)]):
            if lo > cursor:
                ends = [tuple(a[i] + t * (b[i] - a[i]) for i in (0, 1))
                        for t in (cursor, lo)]
                yield f'{side}_{index}', *ends
            cursor = max(cursor, hi)


def solid(model, name, pose, geometry, color):
    """Keep collision and visual geometry identical, including submerged depth."""
    link = element(model, 'link', name=name)
    element(link, 'pose', values(pose))
    collision = element(link, 'collision', name=f'{name}_collision')
    collision.append(copy.deepcopy(geometry))
    visual = element(link, 'visual', name=f'{name}_visual')
    visual.append(copy.deepcopy(geometry))
    material = element(visual, 'material')
    element(material, 'ambient', values(color))
    element(material, 'diffuse', values(color))
    element(material, 'specular', '0.05 0.05 0.05 1')


def validate(config):
    width, depth = config['size']
    if not all(math.isfinite(v) and v > 0 for v in (width, depth)):
        raise ValueError('Arena size must be positive and finite')
    for section, fields in {
        'white_buoy': ('radius',), 'boom': ('width', 'height', 'freeboard'),
        'boundary': ('float_radius', 'float_spacing', 'rope_radius'),
        'corner': ('radius', 'height', 'freeboard'),
    }.items():
        if not all(math.isfinite(config[section][key]) and config[section][key] > 0
                   for key in fields):
            raise ValueError(f'{section}: dimensions must be positive and finite')
        for key in ('color', 'rope_color') if section == 'boundary' else ('color',):
            color = config[section][key]
            if len(color) != 4 or not all(math.isfinite(v) and 0 <= v <= 1 for v in color):
                raise ValueError(f'{section}.{key}: expected RGBA in [0, 1]')
    for section in ('boom', 'corner'):
        if config[section]['freeboard'] >= config[section]['height']:
            raise ValueError(f'{section}: some collision geometry must be underwater')
    names = set()
    for boom in config['booms']:
        if boom['name'] in names:
            raise ValueError('Boom names must be unique')
        names.add(boom['name'])
        a, b = (endpoint(config, boom[key]) for key in ('start', 'end'))
        if math.dist(a, b) < 1e-6:
            raise ValueError(f"{boom['name']}: zero length boom")
        anchors = attached_buoys(boom)
        if not anchors:
            raise ValueError(f"{boom['name']}: every boom must attach to a white buoy")
        for name in anchors:
            if name not in config['buoys']:
                raise ValueError(f"{boom['name']}: unknown buoy {name}")
            if segment_distance(config['buoys'][name], a, b) > (
                    config['white_buoy']['radius'] + config['boom']['width'] / 2):
                raise ValueError(f"{boom['name']}: buoy {name} does not touch boom")
        for x, y in (a, b):
            if not (0 <= x <= width and 0 <= y <= depth):
                raise ValueError(f"{boom['name']}: endpoint outside arena")
    gate_buoys = set()
    gate_sides = set()
    for gate in config['gates']:
        side = gate['side']
        if side not in ('west', 'east') or side in gate_sides:
            raise ValueError('Use one gate per short side (west/east)')
        gate_sides.add(side)
        a, b = [endpoint(config, gate[key]) for key in ('lower', 'upper')]
        x = 0 if side == 'west' else width
        if not (a[0] == b[0] == x and 0 < a[1] < b[1] < depth):
            raise ValueError('Gate buoys must lie on their short edge, lower before upper')
        gate_buoys.update((gate['lower'], gate['upper']))
    radius = config['white_buoy']['radius']
    for name, (x, y) in config['buoys'].items():
        if name in gate_buoys:
            continue
        if not (radius < x < width - radius and radius < y < depth - radius):
            raise ValueError('White buoy overlaps arena boundary')


def generate(config):
    validate(config)
    sdf = ET.Element('sdf', version='1.7')
    sdf.append(ET.Comment(' Generated by scripts/generate_photo_arena.py; edit config/photo_arena.json. '))
    model = element(sdf, 'model', name='kaboat_photo_avoidance')
    element(model, 'static', 'true')
    white = config['white_buoy']
    for name, (x, y) in config['buoys'].items():
        solid(model, f'white_{name}', [x, y, white['center_z'], 0, 0, 0],
              sphere(white['radius']), white['color'])

    boom = config['boom']
    for entry in config['booms']:
        a, b = (endpoint(config, entry[key]) for key in ('start', 'end'))
        # Span the full endpoint centres so no navigable gap appears at a buoy.
        solid(model, f"boom_{entry['name']}",
              [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
               boom['freeboard'] - boom['height'] / 2, 0, 0,
               math.atan2(b[1] - a[1], b[0] - a[0])],
              box([math.dist(a, b), boom['width'], boom['height']]), boom['color'])

    width, depth = config['size']
    corners = [(0, 0), (width, 0), (width, depth), (0, depth)]
    boundary, corner = config['boundary'], config['corner']
    for index, a in enumerate(corners):
        solid(model, f'corner_{index}',
              [*a, corner['freeboard'] - corner['height'] / 2, 0, 0, 0],
              cylinder(corner['radius'], corner['height']), corner['color'])
    for name, a, b in boundary_segments(config):
        length = math.dist(a, b)
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        # Cylinders point along local Z; rotate to horizontal along each edge.
        solid(model, f'boundary_rope_{name}',
              [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0, 0, math.pi / 2, yaw],
              cylinder(boundary['rope_radius'], length), boundary['rope_color'])
        count = math.ceil(length / boundary['float_spacing'])
        for j in range(1, count):
            t = j / count
            solid(model, f'yellow_{name}_{j:02d}',
                  [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), 0, 0, 0, 0],
                  sphere(boundary['float_radius']), boundary['color'])
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    generated = generate(json.loads(args.config.read_text()))
    if args.check:
        if not args.output.exists() or args.output.read_text() != generated:
            parser.exit(1, f'Stale SDF: regenerate {args.output}\n')
        print('Photo arena SDF is up to date')
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(generated)
        print(f'Generated {args.output}')


if __name__ == '__main__':
    main()
