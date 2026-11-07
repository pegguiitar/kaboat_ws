#!/usr/bin/env python3
"""Open one terminal per sensor/detection topic; --start also starts the pipeline."""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    topics = ['/camera/color/image_raw', '/camera/depth/image_raw',
              '/camera/camera_info', '/dock/debug_image', '/detections/dock_marks',
              '/dock/marker_ground', '/dock/detection_status', '/dock/marker_ground_raw']
    commands = []
    if args.start:
        commands.append(('D455 + TensorRT YOLO', [str(repo / 'scripts/run_dock_yolo.sh')]))
    commands += [(topic, [str(repo / 'scripts/watch_dock_topic.sh'), topic]) for topic in topics]
    for i, (title, command) in enumerate(commands):
        subprocess.Popen(['xterm', '-hold', '-T', title, '-fa', 'Monospace', '-fs', '9',
                          '-geometry', f'86x14+{(i % 2) * 660}+{(i // 2) * 240}',
                          '-e', *command], cwd=repo, start_new_session=True,
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)


if __name__ == '__main__':
    main()
